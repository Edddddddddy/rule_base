# ROS 2 适配器契约（L3）

`infrastructure/ros2/` 把公司 `rule_base` 的 `Arm` / `PickArm` / `LifecycleTransport` 端口
映射到 SO-101 的 ROS 2 服务端契约（`arm_msgs` 4 服务 + 标准话题 + TF）。
本文是 WP4 联调（`so101_arm_app`）与 WP5 真机接入的接口说明。

> **2026-11-09 深夜增量（framework v1.5 / F-014、决策 D-093~D-095）**：
> ① **放置位姿改为读 TF**：新增可选 `frames.place`（真机 `base_link -> place_target_01`），
>    profile 删除 `place_position_arm_m` / `place_orientation_arm_xyzw`（保留 approach/retreat
>    clearance）；未配置 `frames.place` 时仍走 legacy「无 place 阶段」行为（向后兼容）。
> ② **每轮开始/结束归零**：`ros2.home_joint_positions` 折叠 rest 经 `<prefix>/joint_command`
>    关节流式下发（D-094），抓取前新增**观察位姿**阶段（`move_to_observation`）。
> ③ **抓取后抬升更高**：真机 `post_grasp_position_arm_m=[0.26,0,0.05]`，并完成 mock
>    `/compute_ik` 可达性复验（D-095）。
> D-090 的「跑 rule_base 前必须手动 `move_to_tcp.py --top-grasp` 预位」前置**作废**
> （`initialize_servo` 不再把当前位姿当 PlanRequest 目标重发，见 §7.6）。
>
> **2026-11-12 增量（S6l，用户现场确认·方案 A）**：抓取成功判定改为**实测闭合宽度**
> （live contact）——夹爪合到 `gripper_closed_m` 后停在中位、未真正全闭即确认夹住物体；
> 固定方块宽度区间只作诊断，不再决定成败。确认后按既有 place 流程继续，不中途松爪丢弃。
> 判据/事件/配置详见 §7.4 第 8 条。

## 1. 默认参数（冻结）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `prefix` | `/arm` | 服务与话题前缀 |
| `base_frame` | `base_link` | 命令与反馈基座；`PlanRequest` 的 `header.frame_id` |
| `tcp_frame` | `gripper_frame_link` | 反馈 TCP 位姿 = TF `base_frame -> tcp_frame`；真机模板 `live_ros2.yaml` 覆盖为 `grasp_tcp_link`（p7 兼容顶抓帧），见 §7.5 |
| `joint_names` | `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` | 6 关节固定顺序 |
| `joint_states` | `<prefix>/joint_states` | `sensor_msgs/JointState`，6 关节 |
| `joint_command` | `<prefix>/joint_command` | `sensor_msgs/JointState`，适配器发布（D-005）；`move_joints`/`gripper` 与**每轮起止归零**（D-094）都走它 |
| `state` | `<prefix>/state` | `std_msgs/Int32`，P7 FSM 编号（D-006） |
| `home_joint_positions` | `[0, 0, 0, 0, 0, 0]`（默认） | 每轮起止归零的折叠 rest（rad，6 关节）；真机/mock 模板为 `[0, -1.52, 1.45, 0.75, 0, 0]`。第 6 位夹爪回零时用**实测关节角**保持、配置值仅占位（D-094，语义见 §7.6） |
| `recover_to_home` | `false` | 恢复路径（`Arm.recover()`）是否额外回零；与每轮 `home_start`/`home_end` 无关，默认关闭 |

参数在配置 `ros2:` 段注入（见 `configs/pick_place/live.yaml`）。话题/服务名由 `prefix` 派生，
无需逐个配置；需要改名前缀即可。

**帧一致性约束（启动即校验）**：`ros2.base_frame` 必须与 `frames.arm_base` 相同。
Pick 规划输出的 TCP 位姿就在 `frames.arm_base` 里，适配器用它作 PlanRequest 的
`header.frame_id`、也用同一帧读 TF 反馈；两者不一致会使位姿被错误解释，因此配置校验直接拒绝。

## 2. 服务与话题

| 接口 | 类型 | 方向 | 适配器用法 |
|---|---|---|---|
| `<prefix>/authority/acquire` | `arm_msgs/srv/AcquireAuthority` | 调用 | `target_topic = <prefix>/joint_command`；返回正 `lease_id` 才算拿到控制权 |
| `<prefix>/authority/release` | `arm_msgs/srv/ReleaseAuthority` | 调用 | 校验 `lease_id`；租约不匹配 → `InactiveLeaseError` |
| `<prefix>/switch_state` | `arm_msgs/srv/SwitchState` | 调用 | 进入 Servo=3；清理回 IDLE=0；失败即抛错，清理未确认不上报成功 |
| `<prefix>/plan_request` | `arm_msgs/srv/PlanRequest` | 调用 | 笛卡尔目标，`header.frame_id = base_frame`；失败 → `ContractError`（业务层走失败分支） |
| `<prefix>/joint_command` | `sensor_msgs/JointState` | 发布 | 6 关节 name/position；`move_joints`（含每轮归零关节流）、`gripper` 使用 |
| `<prefix>/joint_states` | `sensor_msgs/JointState` | 订阅 | 6 关节位置/速度/effort；宽度由夹爪角换算 |
| `<prefix>/state` | `std_msgs/Int32` | 订阅 | 健康检查/IDLE 确认；负值视为故障 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 订阅 | 复用仓库 `TransformBuffer`，含 `tf_age_s`/`tf_skew_s` 门限；场景需要 `arm_base -> object` 与（可选）`arm_base -> place` |

服务端要求（给 WP2/WP4/WP5）：

1. `<prefix>/joint_states` 必须发布 6 维 `velocity` 数组（到位后为 0 也可）。
   业务层用关节速度判定"停稳"（`PickRound._joints_settled`），缺省数组会被视为无证据并超时失败。
2. TF `base_frame -> tcp_frame` 用动态 `/tf` 发布（mock 建议 ≥100 Hz）；`root -> base_frame`
   可以是静态 TF（`/tf_static`，transient-local）。`object` 帧（以及可选 `place` 帧）在真机由
   **独立发布器** `tools/object_pose_publisher.py` 以动态 `/tf` 发布（默认 30 Hz，同一周期同
   stamp 发两帧，见 §8）；集成 mock 服务端可仍用静态 TF。适配器能处理目标帧为静态或动态的情况。
3. `state` 持续发布；`switch_state` 进入 Servo 后要真的把状态切到 3，否则 acquire 阶段会等状态确认超时。

## 3. 端口方法映射

| `Arm` / `PickArm` 方法 | ROS 2 行为 |
|---|---|
| `connect()` | 等 4 个服务 server、`joint_command` 订阅者、`joint_states`/`state`/TF 反馈齐备（受 `limits.warmup_timeout_s` 限制） |
| `acquire()` | `AcquireAuthority`（`manage_lease: true`）+ `SwitchState(3)`；等切到 Servo 的状态确认后才算成功 |
| `feedback()` | `base_frame -> tcp_frame` TF 位姿 + 6 关节位置/速度 + 夹爪宽度/速度/effort；受 `arm.feedback_timeout_s`/`arm.state_timeout_s` 门限 |
| `start_servo(pose)` | 已处于 Servo 时幂等；否则 `SwitchState(3)`。只确认会话并保持当前位姿，**不额外发 PlanRequest 目标**（D-094 后因此不再需要顶抓预位前置） |
| `move(pose)` | `PlanRequest(PoseStamped)`，`frame_id = base_frame`；不可达/超时/拒绝都抛错 |
| `move_joints(positions)` | 发布 6 关节 `joint_command`；每轮 `home_start`/`home_end` 用它流式下发折叠 rest（5 个臂关节 + 实测夹爪角） |
| `gripper(width_m, effort)` | `angle = clamp(width/k, 0, 1.74533)`，随当前关节反馈组成 6 维 `joint_command` 发布；`effort` 只记录日志（D-005 只传位置） |
| `stop()` | 若进入过 Servo：`SwitchState(0)`，并记录 IDLE ack 供 `confirm_idle` |
| `confirm_idle()` | 等 `state == 0` 且样本时间晚于 ack（Press 保留方法；Pick 清理不依赖它） |
| `release()` | `ReleaseAuthority(lease_id)`；租约已失效 → `InactiveLeaseError`，其余拒绝 → `ContractError` |
| `recover(timeout, stable)` | stop → release → 等新鲜反馈稳定 → 重新 acquire；出现 authority 结果未知时拒绝自动重连 |
| `close()` | 销毁本回话的 client/publisher；transport 负责 node/executor/rclpy 生命周期 |

## 4. 夹爪宽度 ↔ 关节角（D-009）

```
k = width_per_rad（默认 0.1 / 1.74533 ≈ 0.05729576）
width_m = clamp(angle_rad * k, 0, max_width_m)
angle_rad = clamp(width_m / k, min_angle_rad, max_angle_rad)
```

适配器与 `so101_arm_app` 服务端必须使用同一组参数；WP4 的 mock 服务端按此约定做接触仿真
（命令 0.03 m 时反馈接触宽度 0.0445 m，满足 live 实测接触窗口
`gripper_closed_m + grasp_contact_margin_m < width < gripper_open_m - margin`；固定方块区间仅供参考）。

## 5. 线程与生命周期

- `Ros2Transport` 是进程内唯一的 rclpy node；在独立线程自旋（`spin_once(timeout_sec=0.05)`）。
- 业务侧 `Arm` 调用发生在 `InstanceRunner` 的单线程执行器里，允许阻塞等待服务响应；
  `rclpy` 回调只在自旋线程执行，二者通过锁/Condition 交换反馈缓存。
- `--transport fake`（默认）**不会 import rclpy**；缺少 ROS 环境时只有 ros2/live 模式失败。
- 关闭顺序：`PickRound` 清理（stop/release/close）→ `InstanceRunner` 调 `transport.close()`
  （停自旋线程 → destroy node → 若由本进程 init 则 `rclpy.shutdown()`）。

## 6. WP4 配置要点

`station/integration/configs/pick_place/mock_ros2.yaml` 需要：

- `frames.root` = mock 发布的 world 帧；`frames.arm_base = base_link`；`frames.object` = 物体帧；
  `frames.place` = 放置点帧（可选；真机/mock 模板都为 `place_target_01`，配置后启用 place 阶段，
  放置位姿从 `frames.arm_base -> frames.place` 动态 TF 读取；不配置则 legacy 无 place 阶段，见 §7.5）；
- `profile.reference_tcp_orientation_root_xyzw` 保持 TCP +X 朝下（`[0, 0.7071, 0, 0.7071]`，
  mock profile 仍用旧姿态；真机 D-047 已改为滚转 180° 的新姿态，见 §7.5），
  这是 `stations/pick_place/planning.py` 的硬校验，属于几何配置，不要改业务校验；
- `profile.post_grasp_position_arm_m` 等目标落在 mock 工作空间内；
- profile 观察位姿字段（D-095）：`observation_clearance_m`（默认 0.06 m，观察位姿在抓取点
  root +Z 上方高度）、`observation_offset_root_m`（默认 `[0,0,0]`；真机 `[-0.039,0.0281,0]`
  为名义腕部相机挂载补偿，见 §7.6）；
- 归零判据（D-094）：`limits.home_tolerance_rad`（默认 0.08 rad）、`limits.home_timeout_s`
  （默认 25 s；真机/mock 模板 30 s）、`ros2.home_joint_positions`（折叠 rest；mock 模板与真机
  同口径 `[0,-1.52,1.45,0.75,0,0]`，见 §7.6）；
- `limits.tf_age_s`/`tf_skew_s` 与 mock TF 频率匹配；`frames.place` 与 `frames.object` 由
  同一发布器在同一周期、同一 stamp 发出，两帧的 TF 老化门限一致；
- `arm.rpc_timeout_ms` 覆盖服务端 PlanRequest 最长耗时；
- `ros2:` 段按上面的默认值，prefix 与 `so101_arm_app` 启动参数一致；
  `ros2.base_frame` 必须等于 `frames.arm_base`（WP2 mock 用 `p7_base_link` 时两边都写 `p7_base_link`）。

### 已实测的 WP2 联调配方（2026-11-05，WP4 可直接复用）

```bash
source ros2_ws/setup_ros_env.sh
# 直接跑安装后的可执行文件，避免 `ros2 run ... &` 被 kill 时留下孤儿节点
APP=ros2_ws/install/so101_arm_app/lib/so101_arm_app/arm_app
ROS_DOMAIN_ID=231 $APP --ros-args -p prefix:=/arm -p backend:=mock \
  -p base_frame:=p7_base_link -p tcp_frame:=gripper_frame_link \
  -p object_frame:=objects_cube_01 &
# rule_base 配置：复用 configs/pick_place/offline.yaml 几何（root 改成 world，
# ros2.base_frame 写 p7_base_link），然后：
ROS_DOMAIN_ID=231 python3 -m rule_base.interfaces.node \
  --config <mock_ros2.yaml> --dry-run --transport ros2 --rounds 2
# 结果：2/2 round_result status=completed（适配器 + 真实 so101_arm_app mock 后端）
```

注意：`ros2 run ... &` 的 `$!` 是 CLI wrapper，直接 `kill $!` 只杀 wrapper，node 会残留并继续
发布同名 TF/服务；WP4 脚本必须按进程组或直接可执行文件 PID 清理（用 `trap`），否则多实例
会把 TF/服务调到别的实例上。

> F-013 起 arm_app 不再接收 `object_frame`（也不再发布 base→object 静态 TF）；复用上面的配方时
> 去掉 `-p object_frame:=...`，物体帧改由 `tools/object_pose_publisher.py` 发布（见 §8）。

## 7. 真机模式（WP5，D-013~D-017；D-047/D-048/D-093~D-095 后更新；D-142 起强制实时视觉）

> **D-142（2026-11-12，用户现场确认）**：真机 `--run` 的抓取坐标只允许来自实时视觉
> `frames.grasp_target`（`configs/pick_place/live_ros2_vision.yaml`）。`live_ros2.yaml` 的
> static 物体位姿（`frames.object` 由 `tools/object_pose_publisher.py` 固定坐标发布）仅保留给
> `--dry-run` / mock / 测试；`--run` 启动即校验 `frames.grasp_target`，缺失则明确失败（不靠配置自觉）。
> **2026-11-14 增量**：env 位置订阅模式（§9，`env_frames.enabled`）消费 env_app 的 live TF 快照，
> 同属允许的 live 位置来源（`validate_live_grasp_source()` 放行）；仅静态 `frames.object` 仍被拒绝。

真机配置：`configs/pick_place/live_ros2_vision.yaml` + `configs/pick_place/profiles/real_pick.yaml`
（保守 limits；`ros2.tcp_frame = grasp_tcp_link`，即 p7 兼容顶抓帧：+X = 手指方向/接近轴、
+Y = 夹爪闭合轴；物体/目标位姿由**视觉节点**以动态 TF 提供（`base_link -> objects_cube_01`、
`objects_cube_01 -> grasp_target_01`，见 `tools/vision/CONTRACT.md`）；旧模板
`live_ros2.yaml` 的 `(0.28, 0, -0.014)` 固定物体点已不参与真机抓取）；顶抓参考姿态按 D-047
绕 TCP 接近轴滚转 180°；完整 place 阶段的放置位姿按 D-093 从 `frames.place`
（`base_link -> place_target_01` 动态 TF）读取，见 §7.5；每轮起止归零与观察位姿按
D-094/D-095 执行，见 §7.6）。几何依据与可达性扫描见 `real_pick.yaml` 文件头与 §7.4；
方块/容器位置必须与 `tools/configs/object_pose_real.yaml` 一致。

### 7.1 同一条 launch 的启动顺序（D-016）

先 mock 硬件验证逻辑，再切真机；两者同一条 launch，只改 `hardware_type`：

```bash
source ros2_ws/setup_ros_env.sh
# 1) mock 硬件 + real 后端（逻辑验证；不动真机；接触仿真走服务端参数）
ros2 launch so101_arm_app arm_stack.launch.py hardware_type:=mock enable_motion:=true \
    mock_grasp_contact_angle_rad:=0.7767
# 2) 真机只读（不使能运动，确认反馈/TF/零指令）
ros2 launch so101_arm_app arm_stack.launch.py hardware_type:=real enable_motion:=false
# 3) 真机可动（只有做动作阶段才用；每个阶段之间停下向用户确认）
ros2 launch so101_arm_app arm_stack.launch.py hardware_type:=real enable_motion:=true
```

> `arm_stack.launch.py` 的几何默认值已与 `live_ros2.yaml` / `real_pick.yaml` 一致：
> `tcp_frame:=grasp_tcp_link`、`static_world_to_base_xyz:=0,0,0`（world 与 base 对齐）、
> `ik_group_name:=manipulator_grasp`。**F-013 起 arm_app 不再接收 `object_frame` /
> `static_object_*`，也不再发布 base→object 静态 TF**；物体位姿由 §8 的独立发布器提供。
> **真机启动不需要再显式覆盖这些值**；若现场改了物体/容器位置，必须同步改
> `tools/configs/object_pose_real.yaml`（object 顶层字段 + `place` 段）与 `real_pick.yaml`
> 并重新做可达性扫描。
>
> **起栈后不再需要手动把臂摆到顶抓姿态**（D-090 的旧前置作废）：rule_base 每轮先关节流式
> 归零到折叠 rest（`home_start`），`initialize_servo` 只确认 Servo 会话、**不重发当前位姿**，
> 因此从 `safe_home.sh` 的 rest/零位直接跑不会被 arm_app 顶抓门控拒绝（见 §7.6）。

launch 必须让 arm_app 的静态 TF/参数与 `live_ros2.yaml` 几何一致（真机无感知；物体帧由 §8 的独立发布器提供）：

| arm_app 参数 | 值 | 说明 |
|---|---|---|
| `prefix` | `/arm` | 与 `ros2.prefix` 一致 |
| `base_frame` / `tcp_frame` / `world_frame` | `base_link` / **`grasp_tcp_link`** / `world` | 与 `frames`、`ros2.tcp_frame` 一致（p7 兼容顶抓帧）；F-013 后不再传 `object_frame` |
| `static_world_to_base_xyz` / `static_world_to_base_quat_xyzw` | `[0,0,0]` / `[0,0,0,1]` | world 与 base 对齐，否则业务层「沿 world +Z 接近/抬升」会变成斜向 |
| `object_frame` / `static_object_xyz` / `static_object_quat_xyzw` | **F-013 已删除** | 物体位姿改由独立发布器提供（§8）；人工按 `tools/configs/object_pose_real.yaml` 放方块 |
| `mock_grasp_contact_angle_rad` | 真机 `0`（默认关）；mock 硬件联调 `0.7767`（↔ 0.0445 m，D-009） | 仅 mock 硬件联调用的接触仿真；真机不得开启 |
| `real_joint_states_topic` / `forward_commands_topic` / `ik_service` | `/follower/joint_states` / `/follower/forward_controller/commands` / `/compute_ik` | D-013/D-014 数据面 |
| `enable_motion` | 阶段 1/2 为 `false`，动作阶段才 `true` | D-015 只读开关 |

### 7.2 安全参数归属（谁负责什么）

| 关注点 | 归属 | rule_base 侧行为 |
|---|---|---|
| 关节速度上限（`max_joint_velocity_rad_s=0.3`） | `so101_arm_app` 内部 100 Hz servo（D-015） | 不自行限速/不绕过；只按 `limits.servo_hz` 刷新笛卡尔目标 |
| 单次目标步长钳位（`max_joint_step_rad=0.08`） | 服务端 PlanRequest（**钳位**语义，Lead 2026-11-05 决策） | IK 成功即 success=true；目标与当前关节差超限时只把伺服目标推到「当前 + 0.08 rad（按方向）」，message 记 `CLAMPED`；rule_base 以 `servo_hz` 重发同一目标逐次收敛。硬拒绝仅：IK 不可达/超 workspace/非顶抓/`enable_motion=false`/无租约或非 SERVO |
| 命令超时（`command_timeout_ms=500`） | 服务端 | rule_base 以 20 Hz 持续刷新目标，低于超时 |
| 工作空间盒 | 服务端参数（默认复用 mock 值） | 业务层另有 `profile.workspace_*` 自检（只判 TCP 反馈） |
| 运动使能 `enable_motion` | 服务端（默认 false） | `--run` 前必须由 launch 显式打开；rule_base 不感知开关，只在 PlanRequest 被拒时失败 |
| 抓取重试 | 业务层 `limits.max_grasp_attempts` | 真机配置固定为 **1**（失败即停，人工确认后再跑） |
| 每轮起止归零（D-094） | 业务层 `ros2.home_joint_positions` + `limits.home_tolerance_rad`/`home_timeout_s` | 经 `<prefix>/joint_command` 关节流式下发折叠 rest（5 臂关节 + 实测夹爪角），不经 PlanRequest、不套 Pick TCP workspace 门；超时即整轮明确失败，不进入抓取重试 |

### 7.3 真机测试阶段（每阶段之间停下确认）

| 阶段 | 内容 | rule_base 参与情况 |
|---|---|---|
| WP5.3 | 只读：joint_states 频率/漂移、TF 可读、零 forward 指令 | 不启动 rule_base；用 `station/integration/real/read_only_check.py` |
| WP5.4 | 单关节 ±0.05 rad 往返（≤0.2 rad/s） | 不启动 rule_base；用 `joint_step_safety.py` |
| WP5.5 | 顶抓 `PlanRequest` 空载（小偏移；`--top-grasp` 已同步 D-047 新滚转姿态） | 不启动 rule_base；用 `plan_request_probe.py` |
| WP5.6 | 抓放方块（人工放方块 + 容器；完整 place 见 §7.5；**跑前不再需要 `move_to_tcp.py --top-grasp` 预位**，rule_base 每轮自行归零→观察→抓取→放置→归零） | 用 `live_ros2_vision.yaml` 起 rule_base（`--run`，非 `--dry-run`；实时视觉目标，D-142），每阶段人确认 |
| WP5.2b 联调 | mock 硬件 + real 后端逻辑验证 | `--dry-run --transport ros2 --rounds 2`（不动真机） |

### 7.4 已知限制 / 真机风险（需 Lead 决策时参考）

1. **抓放阶段不校验臂关节位置**：rule_base 抓放动作只校验 TCP 位姿（TF）、夹爪宽度与关节速度，
   不校验 6 关节角度/跟踪误差；真机安全边界完全靠服务端 D-015（限速/步长/超时/使能）与人工确认。
   例外：**每轮归零阶段**（`home_start`/`home_end`，D-094）按 `limits.home_tolerance_rad` 校验
   5 个臂关节误差（第 6 位夹爪只看实测保持、不参与判据）。
2. **首条 PlanRequest 是大位移（已被钳位语义覆盖）**：rule_base 先 `home_start` 回折叠 rest，
   再把第一个笛卡尔目标定为观察位姿（可能 >10 cm，mock 链在 test 中覆盖）。服务端
   `max_joint_step_rad=0.08` 采用**钳位**而非硬拒绝：IK 成功即 `success=true`，伺服目标每次最多
   前推 0.08 rad 并注明 `CLAMPED`；rule_base 持续重发同一目标逐次收敛（到位由 TF 验证，D-014），
   因此大位移首目标不会导致直接失败。若收敛慢，`limits.stage_timeout_s` 才是需要关注的超时边界。
3. **物体/放置位姿是发布器的常量**（F-013/F-014）：没有视觉/感知，人工必须把方块放到
   `tools/configs/object_pose_real.yaml` 的 object 顶层 `position_m`，把容器放到该配置
   `place.position_m` 对应的台面点；发布器每周期重发同一变换，放偏了只会导致 TCP 到位误差/
   规划失败，不会崩，但也不会自动纠偏（视觉接入后替换数据源）。
4. **command_timeout**：服务端 500 ms 无新目标即保持最后目标；rule_base 若因日志/GC 停顿
   >500 ms，目标会停住（不会发散），恢复后继续刷新。
5. **每轮起止归零（D-094），失败路径不新增失败归零**：成功轮在 `home_start`/`home_end`
   关节流式回折叠 rest；`home_start`/`home_end` 超时或失败直接整轮失败（不进入抓取重试）。
   `ros2.recover_to_home` 仍默认 false；失败清理只做 stop/IDLE → release → close。
6. **真机弱扭矩/跟随滞后**：4.9V 下跟踪滞后明显；`position_tolerance_m=0.01`、
   `orientation_tolerance_rad=0.1` 是保守判定，若到位超时先降幅/降速，不要放宽业务校验掩盖故障。
7. **5 轴顶抓可达集受限（2026-11-09 mock `/compute_ik` 复验；以 `real_pick.yaml` 文件头为准）**：
   顶抓（`grasp_tcp_link` +X 朝下）在 `y=0` 时可达点不是任意 (x,z)。当前真机几何
   （1.5 cm 方块，抓取点 `(0.28,0,-0.014)`，place 模板 `(0.28,0.12,-0.014)`）：
   - `grasp` / `observation` / `post_grasp(z=0.05)` / `place (0.28,0.12,-0.014)` 均 IK 可解；
     `grasp→post_grasp` 1 cm 步进路径全 OK（min margin 0.19 rad）、
     `post_grasp→place_approach→lower_to_place→retreat` 全路径 OK（min margin 0.05 rad）；
     观察位姿余量 0.28 rad（rest/zero 两种种子）。
   - 抬升上限：从 z=0.06 起抬升路径余量只剩 0.13 rad，z≥0.07 不可达 → `post_grasp z`
     定为 0.05（D-095）。
   - **旧几何 5 cm @ `(0.38,0,0.025)` 已弃用**（旧扫描基于当时的 TCP 朝向语义，实测会让
     夹爪水平，不符合顶抓）。
   真机 WP5.6 必须把方块放到发布器配置的 `(0.28,0,-0.014)`、容器放到 place 段模板
   `(0.28,0.12,-0.014)`；否则 `PlanRequest` 会按 D-014 失败（业务层走失败分支，非适配器
   问题）。现场改容器位置后必须重跑 `/compute_ik` 扫描。
8. **抓取成功后判定 = 实测闭合宽度（S6l，用户现场确认 2026-11-12）；固定区间仅诊断**：
   **固定方块宽度区间 `[grasp_min_width_m, grasp_max_width_m]`（1.5 cm 方块 `[0.010, 0.022]` m）
   只用于诊断/日志，不决定抓取成败。** 判定以实测闭合宽度为准：合爪后
   `gripper_closed_m + grasp_contact_margin_m < 实测宽度 < gripper_open_m - grasp_contact_margin_m`
   （夹爪停在中位、未真正全闭）即确认夹住物体；确认后 holding 期间实测宽度与“确认时记录的
   实测接触宽度”偏差不超过 `grasp_width_tolerance_m`，超差仍按既有 lost-grasp 明确失败
   （不放宽容差）。确认成功后按既有 place 流程继续（抬升 → 目标上方 → 下降 → 释放 → 抬升归零），
   **不再中途松爪丢弃**。诊断：`grasp_feedback` 新增 `contact_threshold_m`/`contact_ceiling_m`/
   `contact_width_m`/`grasp_width_tolerance_m`/`width_in_expected_interval`；确认宽度在旧固定
   区间外时额外发 `grasp_width_outside_expected`。现场 15:24 run 实测 7.82 mm 确实夹住物体，
   旧实现按固定区间判失败 → recovery 抬升后回初始位松爪把物体丢下，本改动修复该误判。
9. **mock 硬件联调需要服务端接触仿真**：`backend:=real` 只桥接反馈，mock hardware 空载合爪时
   反馈宽度 = 命令宽度（`gripper_closed_m`），低于 live 接触阈值
   `gripper_closed_m + grasp_contact_margin_m`，永远不会被判为抓取成功；
   实测 `width_m=0.030` → `stage timed out: close_gripper`。
   WP5.2b 起 arm_app real 后端提供 mock 硬件专用参数 **`mock_grasp_contact_angle_rad`**
   （默认 `0`=关闭；mock 硬件联调传 `0.7767` → 0.0445 m，D-009 换算 `angle=width/k`），
   真机保持默认 0。证据见 `artifacts/reports/rule-base-wp5.md` §6。

### 7.5 腕部滚转 180°（D-047）与完整 place 阶段（D-048）

**顶抓参考姿态（D-047）**：真机 `real_pick.yaml` 的 `reference_tcp_orientation_root_xyzw`、
`post_grasp_orientation_arm_xyzw` 与 place 姿态统一改为新滚转姿态；`move_to_tcp.py --top-grasp`、
`plan_request_probe.py --top-grasp` 同步：

```text
旧: [0, 0.7071067811865476, 0, 0.7071067811865476]     # 绕 Y +90°，TCP +X 指向 world -Z
新: [0.7071067811865476, 0, -0.7071067811865476, 0]    # q_旧 ⊗ Rx(180°)
```

新姿态 = 旧姿态绕 TCP 接近轴（+X）再滚 180°：**TCP +X 仍指向 world -Z（接近轴不变）**，
夹爪闭合轴（TCP +Y）从 world +Y 换到 world -Y。原因：腕部相机支架装在腕部一侧，旧姿态下
相机位于夹爪下方、下探/顶抓时贴近台面且看不到夹爪；绕接近轴滚 180° 是保持顶抓几何不变、
把相机换到上方的最小改动。验证方式（mock 栈）：同一目标位姿下腕部相机世界高度应高于
`grasp_tcp`（旧姿态相反）。

**完整 place 阶段（D-048；2026-11-09 修订为 D-093）**：放置位姿不再写在 profile 里，而是从
**可选 TF `frames.place`**（真机 `base_link -> place_target_01`，动态 TF）读取。配置
`frames.place` 即启用 place 阶段；不配置则保持 legacy 行为（无 place 阶段，post-grasp 处
hold + 松爪）。profile 删除 `place_position_arm_m` / `place_orientation_arm_xyzw`，只保留
approach/retreat clearance：

| 字段 | 默认 | 说明 |
|---|---|---|
| `place_approach_clearance_m` | 0.04 | 接近点 = place 同 XY、Z + 该值（`move_to_place_approach`） |
| `place_retreat_clearance_m` | 0.04 | 松爪后沿 root +Z 抬升量（`retreat_from_place`） |

- 放置位姿在规划期由 `frames.arm_base -> frames.place` TF 得到并校验（`planning.py::_place_pose`）：
  必须在 `profile.workspace_*` 内、approach 点也在 workspace 内；root 系下 TCP 接近轴（+X）
  必须朝下、闭合轴（+Y）水平（与抓取对齐门同一条 `limits.max_cube_tilt_rad` 判据）；
  `frames.place` 已配置但 TF 缺失 → `ContractError`，不假装成功。
- 有 place 配置时执行链：`... → move_to_post_grasp → move_to_place_approach（linear）→
  lower_to_place（linear）→ release_gripper → retreat_from_place（linear）→ home_end`；
  `completed` 消息为 `object grasped, moved to place pose, released`。
- 无 place 配置时保持旧行为：`move_to_post_grasp → hold_object（hold_duration_s）→
  release_gripper → home_end`（mock/旧配置零回归）。`hold_duration_s` 只在无 place 时生效。
- 所有 place 段沿用现有 `limits.stage_timeout_s`、servo 限速与到位容差，**未放宽**。
- 失败恢复保持有界：`lower_to_place` 失败时先沿 root +Z 抬升 `pregrasp_clearance_m` 量级
  再按既有恢复返回 post-grasp 位姿并松爪，不新增无界重试；cleanup 语义不变。
- **可达性（2026-11-09 mock MoveIt `/compute_ik` 复验，`arm_stack hardware_type:=mock`）**：
  place 模板点 `(0.28,0.12,-0.014)` 顶抓姿态可解；`post_grasp → move_to_place_approach →
  lower_to_place → retreat_from_place` 全路径 IK OK（min margin 0.05 rad）。现场容器位置改动
  后必须重跑该扫描；真机首测以现场复核为准。

### 7.6 每轮起止归零（D-094）与观察位姿/抬升（D-095）

> 2026-11-09 深夜动作优化（F-014）：用户要求每轮开始/结束都回到折叠 rest、抓取前先到
> 「观察位姿」让腕部相机正对方块、抓取后抬升更高。D-090 的旧前置（跑 rule_base 前用
> `move_to_tcp.py --top-grasp` 手动到顶抓姿态）**作废**。

**每轮阶段顺序（正常完成路径）**：

```text
connect → acquire_control → home_start → initialize_servo → move_to_observation
→ open_gripper → pregrasp → wait_open_gripper → grasp_pose → close_gripper
→ move_to_post_grasp → (place: move_to_place_approach → lower_to_place → release_gripper
   → retreat_from_place | legacy: hold_object → release_gripper)
→ home_end
```

重试路径（`max_grasp_attempts>1` 时）：失败恢复（`retry_*` 阶段）后 `reidentify` →
`open_gripper → pregrasp → ...`；**不再**重新走 `home_start`/`initialize_servo`。

**每轮起止归零（D-094）**：

- `home_start`/`home_end` 都经 `<prefix>/joint_command` **关节流式**下发
  `ros2.home_joint_positions`（真机/mock `[0, -1.52, 1.45, 0.75, 0, 0]`）：前 5 个臂关节用
  配置值，第 6 位夹爪用**实测关节角**保持（回零不动爪，配置值仅占位）。
- 判据：各臂关节绝对误差 ≤ `limits.home_tolerance_rad`（默认 0.08 rad）且持续
  `limits.motion_stable_time_s`；超过 `limits.home_timeout_s`（默认 25 s，真机模板 30 s）
  → `stage timed out: home_start|home_end` 明确失败，不会静默停住。
- 回零阶段**不套用** Pick TCP workspace 门（折叠 rest 本就在抓取工作空间外），但仍受
  Servo 刷新 deadline（`servo_max_gap_s`）约束、可被 Ctrl-C 中断。
- `initialize_servo` 只确认 Servo 会话（`SwitchState(3)`）并保持当前位姿，**不再把当前
  TCP 位姿作为 PlanRequest 目标重发**——这是 D-090 顶抓预位前置作废的原因。
- 失败语义：`home_start`/`home_end` 失败按整轮失败显式上报，**不进入抓取重试/恢复流程**；
  失败路径不新增「失败归零」（清理仍只做 stop/IDLE → release → close）。

**观察位姿（D-095）**：

- 规划期生成（`planning.py::_observation_pose`）：root 系下
  `grasp 点 + observation_offset_root_m + (0, 0, observation_clearance_m)`，姿态固定为
  `reference_tcp_orientation_root_xyzw`（顶抓参考姿态，不随方块 yaw 变化）；再变换回
  `frames.arm_base` 并做 workspace 校验。观察位姿**总是执行**，真机/mock 的差异只在这两个
  profile 字段值：真机 `observation_clearance_m=0.06`、`observation_offset_root_m=[-0.039,0.0281,0]`
  → 观察 TCP≈`(0.241,0.0281,0.046)`；mock profile 用默认值（0.06 / `[0,0,0]`）。
- mock 栈实测（2026-11-09，domain 92，move 到观察位姿 IK 解后读
  `base_link→wrist_camera_optical_frame`）：相机 `(0.2794,-0.0001,0.1205)`，方块
  `(0.28,0,-0.014)`；相机-方块水平偏差 **0.6 mm**、光轴与世界向下夹角 **0.03°**、
  相机-方块距离 **0.1345 m**、off-axis **0.22°**；观察位姿 IK 余量 **0.28 rad**
  （rest/zero 两种种子）。
- 该 offset 是**名义腕部相机挂载补偿**：调 `arm_stack.launch.py` 的 `cam_wrist_xyz/rpy`
  时要同步改 `observation_offset_root_m`，并按现场 2M 画面微调。

**抓取后抬升（D-095）**：

- 真机 profile `post_grasp_position_arm_m` 从 `[0.26,0,-0.01]` 改为 `[0.26,0,0.05]`
  （后撤 2 cm、抬升约 6.4 cm），姿态仍为顶抓滚转姿态。
- mock `/compute_ik` 复验：`grasp→post_grasp(z=0.05)` 1 cm 步进路径全部 IK OK
  （min margin 0.19 rad）；从 z=0.06 起抬升路径余量只剩 0.13 rad、z≥0.07 不可达 →
  0.05 是当前几何的上界附近取值（再高必须换几何/换臂姿态）。

### 7.7 急停（D-017，三层）

1. 在 arm_app / launch 终端 `Ctrl-C`（驱动 on_deactivate 关扭矩）；
2. `bash tools/stop_ros.sh`；
3. `python3 tools/arm_torque.py --off`；每阶段结束确认扭矩已关、机械臂可手动掰动。

## 8. 物体/放置位姿：`tools/` 独立发布器（F-013；place 段 F-014/D-093）

> **D-142（2026-11-12，用户现场确认）**：真机 `--run` 的抓取坐标已强制来自实时视觉
> `frames.grasp_target`（视觉节点 `objects_cube_01 -> grasp_target_01` 动态 TF），
> 本节 `tools/object_pose_publisher.py` 顶层固定的 `objects_cube_01.position_m` **不再参与真机抓取**
> （仅 mock/legacy `--dry-run` 与测试保留）。真机视觉链路的放置点由
> `station/integration/lib/place_target_publisher.py` 读本配置 `place` 段发布（固定 TF 保留）。

用户澄清后的口径（F-013/F-014）：物体初始位姿与放置点目标位姿（pick&place 目标）的发布器
**独立于 rule_base**，位于工作区 `tools/`，是独立脚本节点；`rule_base` 不新增物体/放置坐标
字段，仍只读 TF（`frames.object` 与可选的 `frames.place`）。写死坐标存在**发布器自己的配置**
`tools/configs/object_pose_real.yaml` 里（object 顶层字段 + 可选 `place` 段）；
后续视觉接入时替换发布器/数据源即可，TF 名与语义不变。

### 8.1 启动命令

```bash
source ~/discover/ros2_ws/setup_ros_env.sh

# 1) 离线校验（--dry-run 不 import/初始化 ROS，只打印解析结果）
python3 ~/discover/tools/object_pose_publisher.py \
    --config ~/discover/tools/configs/object_pose_real.yaml --dry-run
# object_pose: base_link -> objects_cube_01 xyz=(0.280,0.000,-0.014) quat=(0,0,0,1) rate=30.0
# place_pose: base_link -> place_target_01 xyz=(0.280,0.120,-0.014) quat=(0.707107,0,-0.707107,0)

# 2) 发布动态 TF（独立终端；与 arm_app / rule_base 使用同一 ROS domain）
python3 ~/discover/tools/object_pose_publisher.py \
    --config ~/discover/tools/configs/object_pose_real.yaml
```

- `--rate HZ` 覆盖配置频率（上限 100 Hz，同时作用于 object/place 两帧）；`--dry-run` 只校验
  配置并打印（有 `place` 段时两行）；正常模式在 SIGINT/SIGTERM 时干净退出（0）。
- 启动顺序：**arm 栈 → 物体发布器 → rule_base（/预检/采集）**；同一 domain 只起一个实例。
- 真机 domain 0 的操作需用户在场 + Lead 授权；mock/测试用隔离 domain（如 231）。

### 8.2 TF 语义与发布器配置

| 项 | 值 |
|---|---|
| parent / child（物体） | 配置 `parent_frame` / `child_frame`（真机 `base_link` / `objects_cube_01`） |
| parent / child（放置点，可选） | 配置 `place.child_frame`（真机 `place_target_01`；parent 与物体相同） |
| topic | `/tf`（动态，`tf2_msgs/TFMessage`；非 `/tf_static`）；有 `place` 段时**每周期同 stamp 发两条** |
| 频率 | 配置 `publish_rate_hz`（默认 30 Hz，两帧同一频率） |
| 坐标 | 物体：`position_m` / `orientation_xyzw`；放置点：`place.position_m` / `place.orientation_xyzw`（m / 单位 xyzw 四元数，parent 系） |

`tools/configs/object_pose_real.yaml`（严格 key；顶层 5 键 + 可选 `place` 段正好 3 键；
缺字段/非法 → 退出码 1）：

```yaml
parent_frame: base_link
child_frame: objects_cube_01
position_m: [0.28, 0.0, -0.014]         # 真机现场取点回填（2026-11-09 实测；模板才是真值）
orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
publish_rate_hz: 30.0

place:                                  # 可选：放置点第二动态 TF（rule_base frames.place）
  child_frame: place_target_01
  position_m: [0.28, 0.12, -0.014]      # 模板占位；现场按容器位置取点回填
  orientation_xyzw: [0.7071067811865476, 0.0, -0.7071067811865476, 0.0]   # 顶抓姿态契约
```

> mock 全链路配置 `tools/configs/object_pose_mock.yaml` 同构：`parent_frame=p7_base_link`，
> object `(0.5208,-0.0269,0.3799)`，place `(0.5208,0.0931,0.3799)` 且
> `orientation_xyzw=[0,0,0,1]`（D-010 mock 几何；mock 的 world→p7_base_link 是 Ry(90°) 基座
> TF，**不要抄真机四元数**）。

### 8.3 rule_base 侧契约（只读 TF，不改代码）

- `frames.arm_base` / `frames.object` 必须与发布器的 `parent_frame` / `child_frame` 一致
  （真机模板 `base_link` / `objects_cube_01` 已一致）；rule_base 只通过 TF 读 `frames.object`。
- 可选 `frames.place` 必须与发布器 `place.child_frame` 一致（真机/mock 模板
  `place_target_01`）；`place` 段表示**放置 TCP 目标位姿**（不是容器几何中心），姿态需满足
  root 系顶抓校验（TCP 接近轴朝下、闭合轴水平，见 §7.5）。
- **不要在 rule_base 配置里写物体/放置坐标**（本仓库配置没有、也不应新增这些位姿字段）；
  坐标的唯一来源是上面的 `tools` 配置（object 顶层 + `place` 段）。
- 发布器未起 / 某帧缺失时，rule_base 在 `limits.warmup_timeout_s` 内因 TF 缺失失败
  （`warmup timed out waiting for valid TF`；配置了 `frames.place` 但 place 帧缺失同样明确
  失败），不会假装成功。重试路径要求 object 帧有**新的动态样本**（`request_new_observation`），
  同一发布器两帧同频发布，不额外等待 place 新样本。
- 视觉接管（T-009）：替换发布器的位姿来源即可，parent/child 与语义不变，rule_base 无需改动。

### 8.4 现场取点回填（`/arm/tcp_pose`，物体 + 放置点两处）

1. 人工摆好方块，把夹爪/臂移到目标顶抓位姿（示教/手拖后保持）；
2. `ROS_DOMAIN_ID=<隔离域> ros2 service call /arm/tcp_pose arm_msgs/srv/GetTcpPose "{}"`
   → 返回 `pose.pose.position` / `pose.pose.orientation`（只读、不动臂、任意 FSM 状态可用）；
3. 回填 `tools/configs/object_pose_real.yaml`：物体取点写顶层 `position_m` / `orientation_xyzw`；
   容器放置点取点写 `place.position_m` / `place.orientation_xyzw`（保持顶抓姿态四元数）；
4. `--dry-run` 复核（应打印 object/place 两行）→ 重启发布器 → 再启动 rule_base；
   改了 place 坐标后必须重跑 `/compute_ik` 可达性扫描（见 §7.5）。

> 真机 domain 0 上的服务调用属于真机操作：必须用户在场 + Lead 授权；mock/测试一律用隔离
> domain（如 231）。

## 9. env 位置订阅模式（2026-11-14，additive，默认关闭）

用户现场架构的第三种坐标来源：不使用 station_support 静态示教、不使用腕部 ArUco 检测，
直接消费 env_app（环境板）发布的位姿 TF；**观察位到位后读一次快照，整轮复用**。

### 9.1 配置（`env_frames:` 段，默认关闭）

```yaml
frames:                       # env 模式不写 object/place/grasp_target（启动即校验拒绝混用）
  root: world
  arm_base: base_link
env_frames:
  enabled: true
  object_tf_prefix: env_marker_   # 物体帧 = <prefix><target_object_id>
  target_object_id: "1"           # 上游指定；环境变量 RULE_BASE_TARGET_OBJECT_ID 覆盖
  place_frame: place_target_01    # env_app 的 /tf_static 放置帧
  grasp_offset_arm_m: [0.0, 0.0, 0.0]  # 现场标定：arm 系固定抓取偏置（marker → 指尖接触点）
  snapshot_timeout_s: 5.0         # 观察位到位后取快照的截止时间（TF 发现/瞬断留余量）
  observe_joint_positions: [...]  # 观察位（5 臂关节，rad）；缺省退化为 TCP 观察位
  observe_joint_tolerance_rad: 0.02
  observe_settle_s: 1.0
  observe_position_arm_m: [0.28, 0.0, 0.20]
  observe_orientation_arm_xyzw: [0.7071, 0.0, -0.7071, 0.0]
```

- 模板配置：`configs/pick_place/live_ros2_env.yaml`（`profile_file: profiles/real_pick.yaml`、
  `vision.enabled:false`、`arm.manage_lease:false`＝实例/FM 持租约）。
- `target_object_id` 可由 `RULE_BASE_TARGET_OBJECT_ID` 覆盖（空值/非法帧 token → 加载即失败）；
  026 launch srv 透传属后续契约变更。
- 互斥校验：`env_frames.enabled` 与 `vision.enabled`/`frames.object`/`frames.grasp_target`/
  `frames.place` 不能同时配置（避免坐标来源歧义）。

### 9.2 TF 与几何契约

| 数据 | 帧/来源 | 说明 |
|---|---|---|
| 物体 | `/tf` 动态 `env_marker_<id>`（parent `base_link`，env_app 约 20 Hz） | 只取**位置**；物体旋转不参与抓取姿态 |
| 放置点 | `/tf_static` `place_target_01`（parent `base_link`） | 与既有 `frames.place` 同一语义（放置 TCP 目标位姿） |
| 抓取 TCP | `marker.position + profile.grasp_contact_offset_tcp_m + env_frames.grasp_offset_arm_m`（前两项在 TCP 系按顶抓参考旋转换算；最后一项是 arm 系固定平移） | 姿态恒为 `profile.reference_tcp_orientation_root_xyzw`（D-047 顶抓常量） |
| pregrasp / observation / post_grasp / place approach/retreat | 沿用 `profiles/real_pick.yaml` 既有逻辑 | 宽度判定仍以实测闭合宽度为准；固定方块区间仅诊断 |

> **`grasp_offset_arm_m` 现场标定（2026-11-14 真机）**：env 只输出「码面」位姿，而顶抓需要
> **指尖接触点在码面下方**（指长/夹持深度）+ 吸收码贴装与外参残差，因此 env 模式必须提供
> 一次性现场偏置。标定方法：重力补偿手拖到正确抓取 TCP（`/arm/tcp_pose` 读数）− 同一时刻
> `env_marker_<id>` 位姿（base_link）＝偏置；必须**每个物体/每个码面尺寸**在真机上重测
> （`|每轴| <= 0.1 m` 由配置校验 fail-closed）。
>
> 2026-11-14 现场两轮实测（id1，码面 2 cm）：
> ① 方块位于 x≈0.23 时：手拖 TCP `[0.2319, 0.0006, -0.0053]`，env `[0.2327, -0.008, 0.004]`
>    → 偏置 `[-0.0008, 0.0086, -0.0093]`；
> ② 方块移到 x≈0.28（配置验证可达区）后：手拖 TCP `[0.2571, -0.0030, -0.0071]`，
>    env `[0.2557, -0.0120, 0.0043]` → 偏置 `[0.0014, 0.0090, -0.0114]`（首轮值）；
> ③ 2026-11-14 晚场次复标（HEAD 437ece0）：`grasp_offset_arm_m` 更新为
>    `[0.0039, 0.0177, -0.0147]`（2026-11-14 窗口值）；真机 env 抓取已成功
>    （width_ok=true，接触宽度≈0.0154 m）。
> ④ 2026-11-15 真机窗口复验：上一版偏置连续两次空夹（合爪宽度≈0.005 m）；按用户现场要求
>    把抓取点**向下移 2 cm** → `[0.0039, 0.0177, -0.0347]`（当前 `live_ros2_env.yaml` 值，
>    2026-11-15 写入）后抓取成功（grasp width≈0.01336 m、width_ok=true；round completed、
>    cleanup_confirmed=true）。偏置仍按物体/码面尺寸逐次标定。
> 偏置的 y/z 分量在 2026-11-14 两轮中稳定（+9 mm / −11 mm 量级），但 env 单码位姿在真实
> 场景中存在 ±5–10 mm 级抖动与几何精度限制（当前外参 `quality_status=review_required`：
> 平移残差 14.3 mm / 旋转 2.81°），这也是 env 模式真机抓取仍需改进标定的原因。

> env 模式**不叠加** `profile.grasp_target_offset_arm_m`（该字段是 vision/teach 目标路径的
> 横向补偿；env 模式示教/检测偏差由 env_app 标定负责）。marker 旋转不参与抓取姿态
> （贴码朝向只影响 env_app 的位姿输出），如需按物体 yaw 调整闭合轴则换 `tcp_xyz/quat` 单物体补偿。

### 9.3 轮次流程与事件（additive）

```text
connect → acquire_control → home_start → initialize_servo
  → env_observe_joints（或 env_observe：TCP 观察位，配置 observe_joint_positions 时用关节空间）
  → settle（observe_settle_s）→ env_snapshot（有界重试直到 snapshot_timeout_s）
  → pick_plan（整轮唯一快照）→ open_gripper → pregrasp → grasp_pose → close_gripper
  → place 链 → home_end
```

- 事件：`env_observe_joints_complete`、`env_snapshot`（`target_object_id/object_frame/place_frame/
  object_pose_arm/place_pose_arm/elapsed_s`）、`env_snapshot_wait`（等待重试诊断，≤1 Hz）、
  `pick_plan`（与 legacy/vision 同字段）。
- 失败语义：TF 缺失/超时 → `env snapshot timed out ...` 整轮失败（stage `env_snapshot`），
  不进入抓取恢复、不夹爪、不伪报成功；`env_*` 阶段失败与 `vision_*` 同档（输入/工作空间失败）。
- 暖机：env 模式 warmup 只等机器链 `root→arm_base`（折叠 rest 时 env 相机可能看不到物体），
  首次快照在观察位读取；`max_grasp_attempts>1` 的重试路径会重新取一次有界快照（真机配置固定 1 次）。
- D-142：env 模式属于 live 位置来源，`--run` 允许（与 vision `frames.grasp_target` 并列）；
  仅静态 `frames.object` 的配置仍被拒绝。
