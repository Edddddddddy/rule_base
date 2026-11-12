# rule_base（SO-101 移植）

公司 `rule_base`（六边形架构工站业务内核）的 SO-101 移植版：**保留 domain / application /
stations/pick_place / transform / fakes / observability 业务内核**，删除公司 CORA 环境依赖
（`infrastructure/cora/`、drawer、press），新写 `infrastructure/ros2/` 作为 **L3 ROS 2 适配层**。
层编号口径与 framework v1.0 一致（L0 `arm_msgs` / L1 `so101_arm_app` / L2 ros2_control+MoveIt /
L3 ROS 2 适配 / L4 业务内核 / L5 站编排 / L6 采集·质检·策略 / L7 Agent·CLI·LeLab），见
[`../../framework/01-总体设计.md`](../../framework/01-总体设计.md) §2。
同一套 Pick 业务代码可在进程内 fake 与真实 ROS 2 服务端之间切换，只由 CLI `--transport` 决定。

| 层 | 内容 | 位置 |
|---|---|---|
| L5 站编排 | Pick + place 规划与执行（顶抓、观察位姿、每轮起止归零、有界重试、清理；place 位姿读 TF，D-093~D-095） | `stations/pick_place/` |
| L4 业务内核 | `InstanceRunner` 轮次状态机（支持 `--rounds N`）；`Arm` / `PickArm` / `LifecycleTransport` / `SnapshotEnvironment` Protocol | `application/runner.py`、`domain/models.py` |
| L3 ROS 2 适配 | `Ros2ArmTopic`（arm_msgs 服务 + joint_command）、`Ros2Transport`（TF/反馈/生命周期） | `infrastructure/ros2/` |
| 离线适配（L4 fake 实现） | `FakePickArm` / `FakePickEnvironment` / `FakeTransport` | `infrastructure/fakes/` |
| 公共 | TF 缓冲、刚体数学、Pick 观测缓存、JSON 事件 | `infrastructure/transform/`、`observability/` |

详细 ROS 2 接口与参数见 [docs/ros2-adapter.md](docs/ros2-adapter.md)；
实现与验证说明见 [docs/develop.md](docs/develop.md)。

## 环境

WSL2 / Ubuntu 24.04 / ROS 2 Jazzy / 系统 python3.12（不建 venv，`setup_ros_env.sh` 会摘掉 venv）：

```bash
source /home/ubuntu/discover/ros2_ws/setup_ros_env.sh
cd /home/ubuntu/discover/station/rule_base_ros2
pip install --user --break-system-packages --no-deps -e .   # 仅首次；让 python3 -m rule_base 免 PYTHONPATH
```

依赖：numpy、pydantic 2、PyYAML（系统 python 已具备）。ROS 2 模式还需要 `arm_msgs` 已编译并 source 工作区。

## 常用命令

```bash
# 1) 默认 dry-run：进程内 fake，单轮，输出 round_result
python3 -m rule_base.interfaces.node --config configs/pick_place/offline.yaml --dry-run

# 2) 连续 N 轮（每轮一条 round_result）
python3 -m rule_base.interfaces.node --config configs/pick_place/offline.yaml --dry-run --rounds 2

# 3) 非 live 但走 ROS 2 适配器（对接 mock/真实服务端，WP4 用；static 模板只允许 dry-run）
python3 -m rule_base.interfaces.node --config configs/pick_place/live.yaml \
    --dry-run --transport ros2 --rounds 10

# 4) live 真机执行：默认 ROS 2 transport（--transport 可省略；必须有服务端在场）
#    D-142：live 抓取只允许实时视觉 frames.grasp_target（live_ros2_vision.yaml）
python3 -m rule_base.interfaces.node --config configs/pick_place/live_ros2_vision.yaml \
    --run --transport ros2

# 5) 真机（仅 Lead+用户在场；先 mock 硬件同栈验证，见 docs/ros2-adapter.md §7）
#    D-142：--run 只允许实时视觉 grasp_target；static 固定物体位姿入口已禁用。
python3 -m rule_base.interfaces.node --config configs/pick_place/live_ros2_vision.yaml \
    --run --transport ros2
```

> `--run` 启动即校验抓取数据源（D-142）：配置没有 `frames.grasp_target`（仅静态 `frames.object`，
> 如 `live_ros2.yaml`）会明确失败并提示改用 `live_ros2_vision.yaml`；`--dry-run`/mock 仍保留
> legacy 静态路径用于测试与 mock 全链路。真机采集入口见 `station/integration/capture/run_capture_real.sh`
> （非彩排 `--source scripted` 必须 `--scene-source vision --target-ids ...`）。

CLI 参数：

| 参数 | 语义 |
|---|---|
| `--config PATH` | 必填；Pick 配置（`task: pick`） |
| `--dry-run` | 默认模式；离线跑 `--rounds` 轮 |
| `--run` | live 执行；省略 `--transport` 时默认 `ros2`（显式 `--transport fake` 会被拒绝） |
| `--transport {fake,ros2}` | `--dry-run` 默认 `fake`（进程内 fake）；`--run` 默认 `ros2`；`ros2` 走 `arm_msgs` 服务 + ROS 2 话题 |
| `--rounds N` | 连续 N 轮（默认 1），每轮输出一条 `round_result`；任一轮失败即退出码 1 |
| `--round-id ID` | 可选：L6 采集编排注入的 round id（与 recorder metadata 同源）；缺省内部 `uuid4().hex`。`--rounds > 1` 时只用于首轮，其余轮次内部生成（向后兼容） |
| `--once` | 兼容保留；轮数由 `--rounds` 控制 |
| `--target` | 已退役（原 Press fixture）；传入会明确失败 |

退出码：`0` 全部完成；`1` 失败/清理未确认；`130` 中断。

## 配置

`configs/pick_place/offline.yaml` 为 fake 几何（公司 P7 测试用坐标系），
`configs/pick_place/live.yaml` 为 SO-101 ROS 2 mock 模板（`world` / `base_link` / `objects_cube_01`），
`configs/pick_place/live_ros2.yaml`（+ `profiles/real_pick.yaml`）为 WP5 **真机/真机栈**保守配置模板
（static 物体位姿；**`--run` 已禁用，D-142**，真机 `--run` 用 `live_ros2_vision.yaml` +
实时视觉 `frames.grasp_target`）。
真实工站配置由调用方按 [docs/ros2-adapter.md](docs/ros2-adapter.md) 提供（真机模式见 §7）。
注意：`ros2.base_frame` 必须等于 `frames.arm_base`（Pick 规划输出的就是该坐标系下的 TCP 位姿，
适配器 TF 反馈也读同一坐标系；不一致会启动即报配置错误）。

真机 `profiles/real_pick.yaml` 的本轮要点（D-047/D-093~D-095，详见 `docs/ros2-adapter.md`
§7.5/§7.6）：

- **顶抓滚转 180°（D-047）**：`reference_tcp_orientation_root_xyzw` / `post_grasp_orientation_arm_xyzw`
  从 `[0, 0.7071, 0, 0.7071]` 改为 `[0.7071, 0, -0.7071, 0]`（绕 TCP 接近轴 +X 再滚 180°）：
  TCP +X 仍朝下、夹爪闭合轴换到另一侧，腕部相机位于夹爪上方（避免下探时相机贴台面）。
- **完整 place 阶段改为读 TF（D-093，修订 D-048）**：放置位姿由 `frames.place`
  （真机 `base_link -> place_target_01` 动态 TF，走 `tools/object_pose_publisher.py` 的
  `place` 段）提供；profile 删除 `place_position_arm_m` / `place_orientation_arm_xyzw`，
  只保留 `place_approach_clearance_m`(默认 0.04)、`place_retreat_clearance_m`(默认 0.04)。
  执行链为 `move_to_post_grasp → move_to_place_approach → lower_to_place → release_gripper →
  retreat_from_place → home_end`；`frames.place` 缺省时保持旧行为（post-grasp 处 hold + 松爪），
  mock 旧配置与旧测试零回归。
- **每轮起止归零 + 观察位姿（D-094/D-095）**：每轮 `home_start`/`home_end` 经
  `<prefix>/joint_command` 关节流式回到折叠 rest（`ros2.home_joint_positions`，第 6 位夹爪用
  实测角保持；判据 `limits.home_tolerance_rad=0.08`，超时 `home_timeout_s=25`，真机模板 30 s）；
  抓取前先到**观察位姿**（抓取点 + `observation_offset_root_m` + root +Z
  `observation_clearance_m=0.06`，固定顶抓参考姿态 → 腕部相机正对方块）；抓取后抬升改为
  `post_grasp_position_arm_m=[0.26,0,0.05]`。D-090 的「跑前必须手动 `move_to_tcp --top-grasp`
  预位」前置**作废**（`initialize_servo` 不再重发当前位姿）。
- **抓取成功判定 = 实测闭合宽度（S6l，用户现场确认；2026-11-12）**：固定方块宽度区间
  `[grasp_min_width_m, grasp_max_width_m]`（1.5 cm 方块 `[0.010, 0.022]` m）**只作诊断**，
  不再决定抓取成败。夹爪合到 `gripper_closed_m` 后停在中位、没有真正全闭
  （`gripper_closed_m + grasp_contact_margin_m < 实测宽度 < gripper_open_m - margin`）即确认
  夹住了物体；确认后 holding 期间允许实测宽度相对“确认时实测接触宽度”漂移
  `grasp_width_tolerance_m`，超差仍按既有 lost-grasp 明确失败。确认成功后按完整 place
  流程继续（抬升 → 目标上方 → 下降 → 释放 → 抬升归零），**不会中途松爪丢弃**；
  close 确认时主动发一条 `grasp_feedback`（`width_ok=True` + 实测宽度）供 L6 判定，
  确认宽度落在旧固定区间外时额外发 `grasp_width_outside_expected` 诊断事件。

ROS 2 相关参数集中在配置的 `ros2:` 段（全部有默认值，可省略）：

```yaml
ros2:
  prefix: /arm                       # 服务/话题前缀（默认 /arm）
  base_frame: base_link              # PlanRequest 目标坐标系 = 反馈位姿基座
  tcp_frame: gripper_frame_link      # 反馈 TF base_frame -> tcp_frame
  joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
  gripper:
    width_per_rad: 0.05729576        # D-009: k = 0.1 / 1.74533
    max_width_m: 0.1
    min_angle_rad: 0.0
    max_angle_rad: 1.74533
  home_joint_positions: [0, 0, 0, 0, 0, 0]   # 默认；真机/mock 模板 [0, -1.52, 1.45, 0.75, 0, 0]（D-094）
  recover_to_home: false             # 恢复路径额外回零；与每轮起止归零无关
```

物体/放置点位姿不在本仓库配置里（F-013/F-014）：`base_link -> objects_cube_01`（物体）与
可选 `base_link -> place_target_01`（放置点，对应 `frames.place`）由独立发布器
`python3 ~/discover/tools/object_pose_publisher.py --config ~/discover/tools/configs/object_pose_real.yaml`
以动态 TF 发布（默认 30 Hz，同周期同 stamp 发两帧；父/子帧与坐标在该配置里），本仓库
`rule_base` 只读 TF（`frames.object` / `frames.place`）；详见
[docs/ros2-adapter.md](docs/ros2-adapter.md) §8。

## 测试

```bash
source /home/ubuntu/discover/ros2_ws/setup_ros_env.sh
cd /home/ubuntu/discover/station/rule_base_ros2

python3 -m pytest tests/unit tests/smoke -q          # 离线单元/冒烟（2026-11-12：378 passed）
ROS_DOMAIN_ID=231 python3 -m pytest tests/integration/test_ros2_loopback.py -q -s
                                                     # 隔离 domain 上跑 ROS2 适配器回环（mock 服务端；1 passed）
```

> 测试文件按 1000 行规则拆分：原 954 行的 `tests/unit/test_pick_place.py` 拆为
> `test_pick_place.py` / `test_pick_place_feedback.py` / `test_pick_place_home_observation.py` /
> `test_pick_place_place.py` + 共享 `pick_place_support.py`（单文件 ≤560 行，公共导入不变）。

集成回环用 `tests/integration/ros2_mock_arm.py` 模拟 arm 服务端（4 服务 + joint_states/state + TF），
只验证适配器接线；WP4 的闸门二使用真实 `so101_arm_app`。

## 已知限制

- 只保留 Pick 一站；Drawer / Press 已随 CORA 一并删除。Pick 做抓取、搬运、可选完整 place
  （接近/下探/松爪/撤离，见 D-048 → D-093，放置位姿读 `frames.place` TF），不含相机识别；
  `frames.place` 缺省时退化为旧的「抓→抬→原地松爪」行为。
- 5 轴臂只支持顶抓姿态；`PlanRequest` 拒绝不可达目标时适配器抛 `ContractError`，业务层走失败/有界重试，绝不假装成功。
- fake 模式是进程内完美跟踪，不代表动力学；ROS 2 模式在 mock 服务端上的通过不等于真机通过（WP5 才有真机）。
- 真机参数（frame 名、夹爪换算、home 位姿、RPC 超时）都通过配置注入；2026-11-09 mock 全链路
  （3/3 completed）已复验每轮归零/观察位姿/place TF 的代码路径，**真机每轮归零运动、观察位姿
  真实相机画面、place 现场取点仍待用户在场验证**。
