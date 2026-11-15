# 实例模式（026 inference instance entry）

> 契约来源：[`docs/plan/contracts.md`](../../docs/plan/contracts.md) **v1.1 §1.2 / §2 / §8**、
> [`docs/plan/03-rule_base-实例化改造开发计划.md`](../../docs/plan/03-rule_base-实例化改造开发计划.md)。
> 本文只说明 rule_base 仓库内的实例入口实现；wire（launch/stop/ready）由 inference manager 提供。
> 证据分档：**mock / 本机**。实例模式**不启真机**，`--dry-run` 仅测试用。

## 1 定位

rule_base 原先只有站侧 CLI（`python3 -m rule_base.interfaces.node --config ...`），由人工/脚本触发。
实例模式下 inference manager 按 026 协议拉起算法程序，因此本仓库新增
`src/rule_base/instance/`：**协议 + keepalive + 子进程 runner + rclpy 入口**。
业务内核（`application/`、`stations/`、`infrastructure/`、`config/`）**零改动**；入口每轮仍用原 CLI
跑一轮（`--rounds 1`），只是由实例协议驱动。

```text
inference_manager ──launch(srv)──▶ 子进程: python3 -m rule_base.instance.entry --config <yaml>
                                        │  stopped(2 Hz, TL) / env/reset_status / arm 话题
                                        └──每轮──▶ 子进程: python3 -m rule_base.interfaces.node
                                                    --config <yaml> --rounds 1 --run
                                                    （mock: --dry-run --transport ros2）
```

## 2 入口命令

```bash
# 真机/生产：由 inference_manager 按 submission.yaml 的 runtime.argv 拉起
# 交付默认 = env 位置订阅（submission.yaml 的 --config 即指向该配置：
# env_app 动态 /tf 物体 + /tf_static 放置点；见 docs/ros2-adapter.md §9）
python3 -m rule_base.instance.entry --config configs/pick_place/live_ros2_env.yaml

# 非默认回退：实时视觉实例配置（teach/vision 入口保留，不删除）
# python3 -m rule_base.instance.entry --config configs/pick_place/live_ros2_vision_instance.yaml

# mock / 测试（禁止真机）：子进程跑 --dry-run --transport ros2
python3 -m rule_base.instance.entry --config configs/pick_place/mock_ros2_instance.yaml --dry-run
```

可选参数（默认值即冻结行为，测试可调）：`--warmup-timeout 30`、`--round-timeout 600`、
`--hold-hz 4`（限 2–5 Hz）、`--stop-grace 5`。

模块结构：

| 文件 | 职责 | ROS 依赖 |
|---|---|---|
| `instance/protocol.py` | env 解析、namespace→话题、`reset_status` False→True 边沿状态机、退出码归类 | 无（可离线单测） |
| `instance/keepalive.py` | hold 命令构造（实测关节位置/夹爪保持）、2–5 Hz 频率校验、start/stop | 无（可离线单测） |
| `instance/runner.py` | 单轮子进程 argv/env、单轮超时、SIGTERM→宽限→SIGKILL、退出码归类 | 无（子进程不导入） |
| `instance/entry.py` | rclpy 入口：env、warmup、stopped 发布、reset 订阅、轮次循环、信号清理 | 有（延迟 import） |
| `tests/integration/instance_entry_mock.py` | 测试包装脚本：`fake_inference_manager --instance-script` 拉起真实 entry | — |

## 3 环境变量（contracts §1.2）

| 变量 | 必填 | 行为 |
|---|---|---|
| `INFERENCE_NAMESPACE` | 否（可空串） | 空串=无前缀；非空必须是绝对名（`/` 开头、无空段）。非法 → 退出码 2 |
| `DOMAIN_ID` | **是** | 必须是整数；进程未继承 `ROS_DOMAIN_ID` 时由它导出；与已继承的 `ROS_DOMAIN_ID` 不一致 → 退出码 2 |
| `STATION_TYPE` | **是** | 本期必须 `pick_place`，否则退出码 2 |
| `CLIENT_ID` / `LEASE_ID` | 否 | 缺失/空 → WARN 结构化日志，不失败（本地 dev）；实例只记录/校验，不 acquire/release |
| `INFERENCE_ID` | 否 | 仅日志（mock manager 注入）；非法值 WARN 后忽略 |

## 4 话题与 QoS（contracts §1.1 / §2）

`namespace = INFERENCE_NAMESPACE`；空串时即下表的绝对名，非空时加 `<ns>` 前缀（无双重斜杠）：

| 话题 | 类型/方向 | QoS |
|---|---|---|
| `<ns>/inference/stopped` | `std_msgs/Bool` 发布（2 Hz 电平） | RELIABLE + TRANSIENT_LOCAL + KEEP_LAST depth 1 |
| `<ns>/env/reset_status` | `std_msgs/Bool` 订阅（2 Hz 电平） | RELIABLE，depth 1 |
| `<prefix>/joint_states` | `sensor_msgs/JointState` 订阅（warmup + hold 实测源） | RELIABLE，depth 10 |
| `<prefix>/joint_command` | `sensor_msgs/JointState` 发布（hold 心跳，6 关节） | RELIABLE，depth 10 |
| `<prefix>/state` 等机械臂服务/话题 | 由业务内核子进程消费 | 见 `docs/ros2-adapter.md` |

`<prefix>` 来自 `--config` 的 `ros2.prefix`（默认 `/arm`）；机械臂话题**不走** `INFERENCE_NAMESPACE`
（contracts §1.1 R4）。`stopped=True` 表示本轮未开始/已结束，`False` 表示正在执行。

## 5 状态机（contracts §2 冻结）

```text
启动 → 读 env/config → rclpy.init → warmup（等 <prefix>/joint_states 首样本，默认 30 s，超时→退出 2）
    → stopped=True（2 Hz 持续发布）
    → 空闲：等待 <ns>/env/reset_status 的 False→True 边沿
    → 边沿：先发 stopped=False → 起子进程跑一轮 → 子进程退出后发 stopped=True → 回空闲
    → 运行期 reset 消息丢弃（不排队、不回放）
    → SIGTERM/SIGINT：终止子进程（SIGTERM→5 s 宽限→SIGKILL）→ 停定时器 → rclpy.shutdown → 退出 0
    → 单轮失败：结构化失败日志 → stopped=True（留 ≥1 发布周期）→ 退出 1（不自动重试）
```

**边沿语义（v1.1 冻结，解决 level-latch 重复触发）**：

1. 启动后必须先观测到至少一次 `False` 才"武装"；
2. 武装状态下收到第一个 `True`（False→True 边沿）= 一次新复位完成 → 消费该边沿、开轮、解除武装；
3. 本轮结束后重新等待 `False`；期间持续 `True` 电平**不得**触发新轮次；
4. 运行期间收到的 reset 消息丢弃。

集成测用 `False` 电平 ≥0.8 s 再 `True` 驱动每轮，并额外验证"第 3 轮后持续 True 2 s 不开新轮"。

## 6 keepalive（D4，租约保持）

- 控制权/租约归 factory manager（D1）；实例配置必须 `arm.manage_lease: false`，实例**不调用**
  `AcquireAuthority`/`ReleaseAuthority`，rule_base 自身只做 `SwitchState(3)`（幂等）。
- 空闲期（warmup 完成后至子进程运行前、以及轮次之间）实例以 **4 Hz（契约 2–5 Hz）** 发布
  `<prefix>/joint_command`：6 关节名/顺序取自配置 `ros2.joint_names`，位置=最近一次实测
  `<prefix>/joint_states` 位置（**夹爪保持**，不改变目标姿态）。无实测样本/样本不完整时跳过该周期。
- **子进程运行期间严格停发**，避免与 rule_base 自身动作流量抢写；本轮结束后恢复。
- 该流量用于 arm_app "先续租、再判门控"（contracts §2 D4）：5 s 无流量会掉租约。
- 集成测每轮结束后**等超过 5 s**（无其它发布者）再调 `/arm/switch_state(0)`（切 IDLE 需有效租约），
  `success=true` 直接证明 keepalive 期间租约未丢。`manage_lease:false`（contracts R14）下实例轮末
  保持 SERVO(3)、由 FM 统一 switch_idle，下一轮 entry 子进程启动时再从 IDLE 切回 SERVO。

## 7 单轮子进程（runner）

| 项 | 值 |
|---|---|
| 默认（live） | `[sys.executable, "-m", "rule_base.interfaces.node", "--config", <abs>, "--rounds", "1", "--run"]` |
| `--dry-run` | `[..., "--config", <abs>, "--rounds", "1", "--dry-run", "--transport", "ros2"]` |
| 子进程 env | 继承当前 env，并把**本仓库** `src` 前置到 `PYTHONPATH`（本机 editable install 指向另一个副本） |
| cwd | 仓库根（`src` 的父目录） |
| 单轮超时 | 默认 600 s（contracts §4 单轮 stopped 600 s）；超时 → SIGTERM → 5 s → SIGKILL → 退出码 1 |
| 退出码归类 | 0=成功；非 0=轮次失败（退出 1）；SIGTERM 中断=退出 0；spawn 失败=失败 |

子进程 stdout/stderr 继承实例 stdout，便于 manager/FM 收集 rule_base 的 JSONL 事件。

## 8 退出码与结构化日志（contracts §2 冻结）

| 退出码 | 含义 |
|---|---|
| `0` | SIGTERM/SIGINT 正常停止（或正常结束） |
| `1` | 运行/轮次失败（含单轮超时、子进程非 0 退出），不自动重试 |
| `2` | 配置/env/warmup/ROS 依赖失败 |

stdout 为单行 JSON（`{"event": ...}`），便于测试/manager 断言；关键事件：

| `event` | 关键字段 | 时机 |
|---|---|---|
| `instance_starting` | namespace/domain_id/station_type/client_id/lease_id/inference_id/dry_run/config | env 解析通过后 |
| `instance_warning` | warning | 可选变量缺失/非法 |
| `instance_topics` | stopped/reset_status/joint_states/joint_command/prefix | 建话题时 |
| `instance_ready` | 同上 + hold_hz | warmup 完成 |
| `round_started` / `round_finished` | round/inference_id（finished 另含 rounds_completed/returncode） | 每轮起止 |
| `round_interrupted` | round/returncode/signal | SIGTERM 中断本轮 |
| `instance_failed` | **stage**/**error**/**inference_id**（+exit_code/round） | 失败退出；`stage ∈ config/environment/warmup/ros2_init/spawn/round/timeout/runtime` |
| `instance_stopping` | signal/rounds_completed/rounds_failed/discarded_resets | 收到 SIGTERM/正常停止 |

## 9 安全边界（不可放宽）

- 实例代码 **不出现** `AcquireAuthority`/`ReleaseAuthority`；`instance` 配置一律 `arm.manage_lease: false`。
- 任何代码/配置 **不写** `/follower/forward_controller/commands`（实例只发 `<prefix>/joint_command`）。
- 默认 `enable_motion`/真机语义不变：`--run` 只在真机授权窗口使用；**本入口的 mock 证据只证明协议/租约/
  清理，不证明真机运动**。
- 实例不做环境 reset、不判定任务结果、不启 recorder；这些由 FM/env_app/recorder 负责。

## 10 本地 mock E2E（domain 229）

```bash
cd /home/ubuntu/yayyrobotics
export ROS_DOMAIN_ID=229                                   # contracts §8 S1 登记域；0 禁用
source ros2_ws/setup_ros_env.sh
export PYTHONPATH=/home/ubuntu/yayyrobotics/rule_base/src:$PYTHONPATH
source /home/ubuntu/yayyrobotics/inference_manager/outputs/ros2-interfaces/install/setup.bash
                                                           # inference_manager_msgs（IM 唯一 owner；最后 source）
cd rule_base
python3 -m pytest tests/integration/test_instance_entry.py -q -s
```

测试用既有资产（只读，不改）：`tests/integration/ros2_mock_arm.py`（arm 双 + 租约）、
`factory_manager/local/integration/mocks/fake_inference_manager.py --instance-script tests/integration/instance_entry_mock.py`
（真实 entry 通过包装脚本被拉起）。断言：ready/stopped 时序、3 轮 reset 边沿、每轮 >5 s 空闲后
switch_state 3→0 成功（keepalive 续租证据）、hold 频率 2–5 Hz 且位置=实测、持续 True 不重开轮、
stop 后 entry exit 0、SIGTERM manager exit 0、无残留进程。

前置缺失时测试给出 skip/fail 提示（未 source ROS → skip；domain 非 229 或 0 → fail）。

## 11 已知限制 / 后续

1. **每轮一个子进程**：每轮有 CLI 启动 + ROS 连接 warmup 开销（秒级）；库内复用会话是后续优化项，
   不影响 026 协议。
2. warmup 只检查 `<prefix>/joint_states` 首样本（契约要求的最小连接检查）；更完整的服务/相机就绪由
   FM/真机前置（station_support）保证。
3. 实例与 manager 之间只有 026 控制面；实例失败只通过退出码 + 结构化日志归因，不做重试。
4. mock 证据（domain 229）通过 **不代表** 真机通过；真机 Gate 需用户在场 + 授权（S5）。
5. 同一 domain 上不要并行跑两个实例 E2E（控制面话题固定，会互相干扰）；各会话使用 §8 登记的独立 domain。
