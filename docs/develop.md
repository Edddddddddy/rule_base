# 开发说明（SO-101 移植）

本仓库由公司 `rule_base` 移植而来：业务内核（domain / application / stations / transform /
fakes / observability）保持六边形架构与语义，只把原 `infrastructure/cora/` 适配器替换为
`infrastructure/ros2/`。接口契约见 [ros2-adapter.md](ros2-adapter.md)，入口与常用命令见
[README](../README.md)，AI 修改约定见 [AGENTS.md](../AGENTS.md)。

## 1. 移植范围

删除（公司工装/CORA 专用）：

- `infrastructure/cora/`（TF/臂 topic 适配、原生 reset/stopped 通道、依赖校验）；
- `stations/drawer/`、`stations/press/`、`configs/drawer/`、`configs/press/`；
- `deps/`、`packages/cora-*.whl`、`uv.lock`、`submission.yaml`、`tools/`（CORA 联调工具）；
- CORA 相关配置模型（`CoraSettings`、manifest 校验、`live_preflight`、`runtime_environment`）；
- CORA 专属测试（`test_arm_topic*.py`、`test_cora_*`、drawer/press 测试、native loopback）。

保留并适配：

| 路径（`src/rule_base/`） | 职责 |
|---|---|
| `domain/models.py` | 传输无关的 Pose/Feedback/PickScene/RoundResult 与 Arm/PickArm/LifecycleTransport Protocol |
| `application/runner.py` | `InstanceRunner`：transport 生命周期、TF 预热、单轮/多轮、心跳、清理与退出码 |
| `stations/pick_place/` | 规划（顶抓、目标姿态对齐、观察位姿、place 读 TF、工作空间）与执行（每轮起止归零、Servo 目标流、夹持确认、有界重试、清理） |
| `infrastructure/ros2/` | `Ros2Transport`（node/TF/反馈/生命周期）+ `Ros2ArmTopic`（arm_msgs 服务与 joint_command） |
| `infrastructure/transform/` | `TransformBuffer`（受限 TF 树、时效/偏斜/冲突门限）、刚体数学、`PickObservationCache` |
| `infrastructure/fakes/` | 离线 `FakePickArm`/`FakePickEnvironment`/`FakeTransport`（fake dry-run 用） |
| `config/` | 严格 pydantic 配置：Pick 几何、limits、`arm`、`ros2` 段 |
| `interfaces/node.py` | CLI：`--config/--dry-run/--run/--transport/--rounds/--once/--target(拒绝)`；`--run` 省略 `--transport` 时默认 `ros2`（显式 `fake` 被拒绝） |
| `observability/events.py` | JSON 事件日志（`startup`/`armed`/`stage`/`round_result`/`shutdown` 等） |

## 2. 一轮 Pick 的时序（fake 与 ros2 相同）

```text
transport.open() + 环境 TF 预热（InstanceRunner）
→ PickRound.run():
   validate：snapshot PickScene（arm_base←object 与可选 arm_base←place）、
            plan_pick（grasp/pregrasp/observation/place + 工作空间/姿态/宽度校验）
   attempt 1：connect → acquire（租约 + Servo=3）
      → home_start（关节流式回折叠 rest；失败即整轮失败，不进入抓取重试）
      → initialize_servo（只确认 Servo 会话，不重发当前位姿）
      → move_to_observation（腕部相机到方块上方观察位姿，D-095）
      → open_gripper → pregrasp → wait_open_gripper → 直线下降到 grasp
      → close_gripper（宽度确认）→ 直线抬升到 post_grasp（D-095：z=0.05）
      → place（配置了 frames.place）：move_to_place_approach → lower_to_place
         → release_gripper → retreat_from_place
         = 或 legacy（未配置 frames.place）：hold_object（持续刷新夹爪）→ release_gripper
      → home_end（回折叠 rest，D-094）
   失败：有界恢复（重新观测/回位/Servo 会话重建，最多 max_grasp_attempts 次；
         重试路径 reidentify 后直接 open_gripper，不再回零/重新观察；
         home_start/home_end 失败不进入恢复/重试）
   finally：stop（IDLE）→ release（租约）→ close（本轮回话 client/订阅）
→ transport.close()（node/线程/rclpy 生命周期）
```

> 2026-11-09 增量（F-014/D-093~D-095）：place 位姿从 `frames.place` TF 读取（profile 不再写
> 放置坐标）；每轮起止归零 + 抓取前观察位姿 + 抓取后抬升 0.05 m；D-090 的
> 「跑前必须手动 `move_to_tcp --top-grasp` 预位」前置作废。

`--rounds N` 由 `InstanceRunner` 串行执行 N 次 `cycle`；每轮重新创建 Arm 适配器、重新 acquire/release，
每轮输出一条 `round_result`；任一轮 failed → 退出码 1，中断 → 130。

## 3. ROS 2 transport 设计要点

- **单 node、单自旋线程**：`Ros2Transport.open()` 创建 node，`SingleThreadedExecutor.spin_once(0.05)`
  跑在守护线程；`Arm` 端口调用发生在 `InstanceRunner` 的 worker 线程（允许阻塞等服务响应）。
- **反馈缓存**：`/tf`、`/tf_static`、`<prefix>/joint_states`、`<prefix>/state` 的订阅都在 transport，
  环境快照与 Arm 反馈共用同一份缓存，避免两处各自订阅导致的不一致。
- **时效门限**：TF 用 `limits.tf_age_s`/`tf_skew_s`；关节/状态用 `arm.feedback_timeout_s`/
  `arm.state_timeout_s`；TCP 位姿额外检查最近一次动态 TF 的接收时间。
- **不能假装成功**：PlanRequest 拒绝/超时、switch 拒绝、release 拒绝都会抛 `ContractError`
  或其子类；`InstanceRunner` 只在 `round_result.cleanup_confirmed` 为真时报告成功退出。
- **确定失败语义**：acquire 超时会把本轮标记为 authority 结果未知，`stop()` 不再发 IDLE，
  清理未确认 → round failed；release 遇到租约已失效抛 `InactiveLeaseError`。

## 4. 验证

```bash
source /home/ubuntu/discover/ros2_ws/setup_ros_env.sh
cd /home/ubuntu/discover/station/rule_base_ros2

# 离线单元 + 冒烟（fake 路径，不依赖 ROS 图）
python3 -m pytest tests/unit tests/smoke -q

# ROS 2 适配器回环（隔离 domain 231；mock 服务端 + 真实 CLI）
ROS_DOMAIN_ID=231 python3 -m pytest tests/integration/test_ros2_loopback.py -q -s

# 行为冒烟
python3 -m rule_base.interfaces.node --config configs/pick_place/offline.yaml --dry-run
python3 -m rule_base.interfaces.node --config configs/pick_place/offline.yaml --dry-run --rounds 2
```

`tests/integration/ros2_mock_arm.py` 只用于验证适配器接线（4 服务 + 反馈 + TF + 完美跟踪），
**不是** WP4 的 mock 服务端；闸门二以 `so101_arm_app` 为准。

## 5. 未验证 / 后续

- 真机（WP5）：frame 名、夹爪换算、home 位姿、RPC 超时均未在真实 SO-101 上验证；真机前需先跑
  mock 与只读反馈。
- 五轴顶抓：`PlanRequest` 只支持顶抓；不可达必须失败（服务端负责返回 `success=false`）。
- `confirm_idle` 为 Press 保留方法，当前 Pick 清理路径不依赖它；真机接入时若服务端 IDLE 确认有语义变化需复核。
- 采集/质检/策略闭环（WP6–WP8）不在本仓库。
