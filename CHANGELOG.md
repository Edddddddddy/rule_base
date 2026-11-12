# Changelog

## Unreleased

- **2026-11-12 · 抓取确认改为实测闭合宽度（S6l，用户现场确认，方案 A）**：
  - **问题（现场 15:24 run，target id2）**：夹爪实测 7.82 mm（确实夹住 1.5 cm 方块），但被固定
    方块宽度区间 `[0.010, 0.022]` m 判失败 → 走 recovery 抬升后回初始位松爪，把物体丢下。
  - **行为**：`PickRound._grasped()` 改为 **live 实测接触判据** —— 未确认前
    `gripper_closed_m + grasp_contact_margin_m < 实测宽度 < gripper_open_m - grasp_contact_margin_m`
    （夹爪停在中位、未真正全闭即认为两指间有物体）；确认后 holding 期间
    `|实测宽度 - 确认时记录的实测接触宽度| <= grasp_width_tolerance_m`，超差仍按既有
    lost-grasp 明确失败（不放宽容差）。确认成功即进入既有 place 流程
    （抬升→目标上方→下降→释放→抬升归零），**不再中途松爪丢弃**；place 顺序、超时、
    `grasp_target_offset_arm_m` 与失败语义均未变。固定方块区间
    `[grasp_min_width_m, grasp_max_width_m]` **只作诊断/日志**，不参与成败判定。
  - **动作层强制实时视觉（D-142，用户现场确认）**：`application/runner.py` 新增 `validate_live_grasp_source()`——`--run`（live）启动即要求 `frames.grasp_target`；只有静态 `frames.object` 的配置（如 `live_ros2.yaml`）会明确失败并提示改用 `live_ros2_vision.yaml`，`--dry-run`/mock 保留 legacy 静态路径（测试/mock 全链路不受影响）。新增 2 项 runner 单测（拒绝 static live 配置、vision 配置通过）。
  - **事件（只增不改）**：close 确认成功后主动发一条 `grasp_feedback`（`stage=close_gripper`、
    `width_m=实测接触宽度`、`width_ok=True`，带 `stable_for_s`/`required_stable_s`/`elapsed_s`），
    修掉 L6 `capture_rounds.judge_scripted` 只采到 close 起点（≈张开宽度 0.0799）导致
    task_success=false 的误判；`grasp_feedback` 诊断字段扩展
    `contact_threshold_m`/`contact_ceiling_m`/`contact_width_m`/`grasp_width_tolerance_m`/
    `width_in_expected_interval`；确认宽度在旧固定区间外时额外发诊断事件
    `grasp_width_outside_expected`。
  - **配置**：`PickProfile` 新增 `grasp_contact_margin_m`（默认 0.0015，gt=0/le=0.02）与
    `grasp_width_tolerance_m`（默认 0.002，gt=0/le=0.02）；model_validator 校验
    `closed+margin < open-margin`、`margin <= grasp_min_width_m - gripper_closed_m`、
    `tolerance < open-closed`。现有 `configs/pick_place/profiles/*.yaml` 全部用默认值通过校验。
  - **测试**：改写旧固定区间语义的断言（thin contact 0.033、live 边界、45° 投影区间仅诊断），
    新增：空夹仍失败（闭合宽度低于阈值）、薄接触 0.033 完成整轮并走完整 place 链
    （+`grasp_width_outside_expected`）、真机数值回归（closed=0.005/min=0.010/max=0.022/
    cube=0.015/open=0.08 + `object_width_m=0.00782` → completed 且最后一条 `grasp_feedback`
    `width_ok=True`/`width_m≈0.00782`）、夹爪卡在全开（0.08）close 超时失败、确认事件字段、
    新配置校验；lost-grasp/瞬态坏样本测试保留（0.07 注入仍判丢失/可容忍）。
    顺带修正 `test_pick_holds_at_configured_post_grasp_pose_before_releasing` 的浮点累加
    断言（`>= 5` → `>= 5 - 1e-9`：补丁时钟 0.01 s 步进累加可能舍入到 4.999999999999996，
    属既有测试脆弱性，覆盖语义不变）。
    `pytest tests/unit tests/smoke` → **378 passed**（基线 355 passed + 1 failed；其中 1 条为
    对齐用户 WIP 的陈旧断言，见下；本机连续 2 次全量复跑均 378 passed）。`ruff check src tests`
    无新增错误（仅拆分测试文件既有 I001）。`mypy` 两个改动源文件通过；`execution.py` 858 行、
    `config/pick_place.py` 418 行（≤1000）。
  - **用户 WIP 陈旧断言/注释对齐（非本次行为变更）**：`tests/unit/test_pick_vision_config.py`
    原先期望 `grasp_target_offset_arm_m=(0, 0.015, 0)`，与当前用户 WIP 真机 profile
    `profiles/real_pick.yaml` 的 `[0, 0.020, 0]` 不一致（改动前即失败）；改为直接读取该
    profile 文件比较并锁定 `(0, 0.020, 0)`；`configs/pick_place/live_ros2_vision.yaml` 注释
    的 `[0, 0.015, 0] / 1.5 cm` 同步改为 `[0, 0.020, 0] / 2 cm`（配置值/语义未动）。
  - **文档**：`README.md` 真机要点新增实测宽度确认说明；`docs/ros2-adapter.md` 顶部增量、
    §4 D-009 接触仿真说明、§7.4 已知限制里“抓取判定区间”表述改为“固定区间仅诊断，判定以
    实测闭合宽度为准（用户现场确认）”。
- 从公司 `rule_base` 移植为 SO-101 Pick 站业务内核：删除 `infrastructure/cora/`、Drawer/Press 工站、
  CORA 依赖与联调工具/测试，保留 domain、application、stations/pick_place、transform、fakes、observability。
- 新写 `infrastructure/ros2/`：`Ros2Transport` 单 node/自旋线程 + TF/反馈缓存 + 生命周期，
  `Ros2ArmTopic` 实现 `Arm`/`PickArm` Protocol（arm_msgs 4 服务、`<prefix>/joint_command`、TF 位姿反馈）。
- 装配改为 fake/ros2 双 transport：`--dry-run --transport ros2` 在非 live 模式下走真实 ROS 2 适配器，
  `--run` 强制 ros2；新增 `--rounds N`（每轮一条 `round_result`）。删除 CORA 的 manager reset/stopped 常驻模式。
- 配置：删除 `CoraSettings`/manifest 校验/live preflight/`runtime_environment`；新增 `ros2` 段
  （prefix/base_frame/tcp_frame/6 关节名/夹爪换算/home 位姿），默认 `/arm`、`base_link`、`gripper_frame_link`。
- 夹爪宽度↔关节角按 D-009 线性换算（k=0.1/1.74533，`width=clamp(angle*k,0,0.1)`），适配器与服务端共用参数。
- 测试：保留 Pick 几何/执行/重试测试并适配新配置；新增 config/ros2 参数、CLI、仓库契约冒烟测试，
  以及 `tests/integration/test_ros2_loopback.py`（隔离 domain 231，mock arm 服务端跑通 2 轮 completed）。
- 配置帧一致性：`ros2.base_frame` 必须等于 `frames.arm_base`（启动即校验），避免规划位姿与 TF 反馈读不同基座。
- 重试观测修复：`PickObservationCache.request_new_observation()` 不再清空共享 TF 动态边（同一 buffer 还承载
  `base_frame -> tcp_frame` 臂反馈；改为只按物体样本时间戳要求新观测，静态物体帧直接复用）。
- CLI 新增可选 `--round-id`（S2-5）：L6 采集编排在 `/recorder/start` 前生成 `collection_round_id`
  并把同一 id 传给 rule_base，保证 `round_result.round_id` == recorder metadata `round.id` ==
  run.json `round_id`；缺省仍内部 `uuid4().hex`（向后兼容），`--rounds > 1` 时只用于首轮。
- 文档：README/AGENTS/docs 重写为 SO-101 移植版；新增 `docs/ros2-adapter.md`（WP4/WP5 接口契约）。
- 已用真实 `so101_arm_app` mock 后端（非本仓库 mock）在隔离 domain 231 上跑通 2/2 轮
  `--dry-run --transport ros2`，WP4 配方见 `docs/ros2-adapter.md`。
- WP5 真机准备：新增 `configs/pick_place/live_ros2.yaml` + `profiles/real_pick.yaml`
  （保守 limits：20 Hz 目标刷新、1 cm 步进、5 cm/s、到位容差 1 cm/0.1 rad、单次抓取不重试）；
  物体占位最终定为 `(0.38, 0, 0.025)`（IK 可达扫描挑出：pregrasp/grasp 关节限位余量 0.84/0.54 rad，
  真机启动必须显式传 `static_object_xyz:=0.38,0,0.025`）；
  `docs/ros2-adapter.md` §7 真机小节（同栈启动顺序、安全参数归属、阶段化测试、可达性/接触仿真已知限制、急停）。
- 适配器容错：`<prefix>/joint_states` 的 effort 非有限（硬件无 effort state interface 时
  joint_state_broadcaster 输出 NaN）按诊断字段不可用处理，打一次性 `arm_feedback_effort_unavailable`；
  position/velocity/TF 仍严格有限校验。
- WP5.2b 联调（arm_app `backend:=real` + `hardware_type:=mock`，domain 232）：
  `live_ros2.yaml` 原样 `--dry-run --transport ros2 --rounds 2` → **2/2 completed、cleanup 确认、无 runtime_error**
  （证据见 `artifacts/reports/rule-base-wp5.md` §6.5）。
- D-047 腕部滚转 180°：真机 `profiles/real_pick.yaml` 的 `reference_tcp_orientation_root_xyzw` /
  `post_grasp_orientation_arm_xyzw` / place 姿态改为 `[0.7071, 0, -0.7071, 0]`（= 旧姿态
  `[0, 0.7071, 0, 0.7071]` 绕 TCP 接近轴 +X 再滚 180°：接近轴仍指向 world -Z、闭合轴换到另一侧，
  腕部相机位于夹爪上方，避免下探时贴台面）；`station/integration/real` 的 `move_to_tcp.py`
  （`--top-grasp`）与 `plan_request_probe.py`（`--top-grasp`）顶抓常量同步，默认 tcp frame
  改为 `grasp_tcp_link`（arm_stack 真机默认）。
- D-048 完整 place 阶段：`PickProfile` 新增可选 `place_position_arm_m` /
  `place_orientation_arm_xyzw` / `place_approach_clearance_m`(0.04) / `place_retreat_clearance_m`(0.04)；
  `_finish_attempt` 在有 place 配置时执行 `move_to_post_grasp → move_to_place_approach →
  lower_to_place → release_gripper → retreat_from_place`（均为 linear；`completed` 消息更新为
  "object grasped, moved to place pose, released"），无 place 字段时保持旧的 post-grasp
  hold + 原地松爪行为（旧 mock 配置/205+ 测试零回归）；`lower_to_place` 失败恢复先沿 root +Z
  抬离台面再按既有有界恢复返回 post-grasp 后松爪，不新增无界重试。
- 真机 `profiles/real_pick.yaml` 配齐 place 字段（默认 `(0.28, 0.12, -0.03)`，现场按容器位置改），
  `calibration_revision` → `so101-real-top-grasp-small15-x028-low-roll180-place-v7`；本轮用 mock
  MoveIt `/compute_ik` 自查：新滚转姿态在 grasp 点与 place 点均 IK 可解（FK 残差 <1 mm，
  新姿态 wrist_roll≈-1.49 rad、place≈-1.96 rad，限位余量 ≥0.78 rad），真机首测仍按现场复核。
- 测试：新增顶抓滚转四元数数学断言（`q_new = q_ref ⊗ Rx(180°)`，接近轴不变/闭合轴翻转）、
  place 阶段序列与目标位姿（fake arm）、无 place 字段时的旧行为回归、place 配置解析/校验；
  修正 `tests/unit/test_config.py::test_live_ros2_real_template_is_conservative` 的陈旧
  `tcp_frame` 断言（冻结合同为 `grasp_tcp_link`）。`tests/unit tests/smoke` 221 passed；
  mock chain 10/10 不变。
- 文档：README 配置说明/已知限制、`docs/ros2-adapter.md` §7 真机几何（1.5 cm 物体 @
  `(0.28,0,-0.03)`、`grasp_tcp_link`、新增 §7.5 滚转与 place）、
  `station/integration/real/README.md` 真机 SOP 同步为本轮几何。
- F-013 修正（docs-only，无代码变更）：物体初始位姿发布器归工作区 `tools/` 独立节点
  `python3 ~/discover/tools/object_pose_publisher.py --config ~/discover/tools/configs/object_pose_real.yaml`
  （动态 TF，默认 30 Hz；parent/child 与坐标在发布器自己的配置里；现场用只读 `/arm/tcp_pose`
  取点回填）。`rule_base` 不新增配置字段、业务代码不变，仍只读 TF `frames.object`；本仓库
  `docs/ros2-adapter.md` §2/§6/§7/§8 与 `README.md` 同步说明该来源与只读契约，
  `tests/integration/config_real_backend_mock.yaml`（legacy、未被测试引用）头注释指向新发布器。
- **2026-11-09 深夜 · rule_base 动作优化（F-014 / D-093~D-095）**：
  - **放置位姿改为读 TF（D-093）**：`PickFrames` 新增可选 `frames.place`；`PickScene` 新增
    `arm_to_place`、`PickPlan` 新增可选 `place`；`planning._place_pose()` 从
    `frames.place` 读放置位姿并做 workspace + root 系顶抓姿态校验（TCP +X 朝下、闭合轴水平）。
    profile **删除** `place_position_arm_m` / `place_orientation_arm_xyzw`，保留
    `place_approach_clearance_m` / `place_retreat_clearance_m`；未配置 `frames.place` 时
    保持 legacy 行为（无 place 阶段：post-grasp hold + 松爪），旧 mock 配置零回归。
  - **每轮起止归零（D-094）**：`PickLimits` 新增 `home_tolerance_rad=0.08` / `home_timeout_s=25`；
    `ros2.home_joint_positions` 经 `<prefix>/joint_command` **关节流式**下发（前 5 臂关节用配置值、
    第 6 位夹爪用实测角保持，回零不动爪）；回零阶段不套用 Pick TCP workspace 门、可中断；
    位置误差 + 稳定时间判据，超时明确失败且不进入抓取重试。`initialize_servo` 不再把当前
    TCP 位姿当 PlanRequest 目标重发 → D-090 的「跑前手动 `move_to_tcp --top-grasp` 预位」作废。
  - **抓取前观察位姿（D-095）**：`PickProfile` 新增 `observation_clearance_m`（默认 0.06）/
    `observation_offset_root_m`（默认 `[0,0,0]`）；`planning._observation_pose()` 在 root 系
    取「抓取点 + offset + root +Z*clearance」，姿态固定 `reference_tcp_orientation_root_xyzw`，
    执行序列新增 `move_to_observation`（在 open_gripper 之前）。
  - **抓取后抬升更高（D-095）**：真机 `post_grasp_position_arm_m` 从 `[0.26,0,-0.01]` 改为
    `[0.26,0,0.05]`。
  - **测试**：`/usr/bin/python3 -m pytest tests/unit tests/smoke -q` → **259 passed**
    （基线 221，+38；2026-11-09 Lead 复跑）；`ROS_DOMAIN_ID=231 ... test_ros2_loopback.py -q` →
    **1 passed**（16.11 s，Lead 复跑）；mock 全链路（隔离域 91，`run_mock_chain.sh --rounds 3`）
    **3/3 completed**、每轮事件序列含 `home_start`/`move_to_observation`/`lower_to_place`/
    `home_end`。原 954 行 `tests/unit/test_pick_place.py` 按 1000 行规则拆为
    `test_pick_place{,_feedback,_home_observation,_place}.py` + `pick_place_support.py`
    （单文件 ≤560 行，公共导入不变）。
  - **配置**：真机 `live_ros2.yaml` 新增 `frames.place: place_target_01`、
    `ros2.home_joint_positions: [0.0,-1.52,1.45,0.75,0.0,0.0]`、`home_tolerance_rad`/
    `home_timeout_s`（30 s）；`real_pick.yaml` `calibration_revision` →
    `so101-real-top-grasp-small15-x028-low-roll180-place-v8`、观察位姿/抬升值；
    mock 链配置 `station/integration/configs/pick_place/mock_ros2.yaml` 同步 `frames.place`
    + `home_joint_positions`。
  - **文档**：`README.md`、`docs/ros2-adapter.md`（§1/§2/§3/§6/§7/§8 + 新增 §7.6）、
    `docs/develop.md` §1/§2；工作区 `docs/50` §0.10、`docs/51` §4~§6、`framework/07` F-014、
    `PLAN.md` 第十七部分、`api.md`、根/本仓库 `AGENTS.md`、
    `artifacts/reports/rule-base-action-opt-2026-11-09.md`。
- **2026-11-10 · 视觉目标帧 + 有界视觉伺服（T-009/T4，契约 `tools/vision/CONTRACT.md` §6）**：
  - **配置**：`PickFrames` 新增可选 `frames.grasp_target`（与现有帧名去重校验）；`PickSettings`
    新增可选 `vision` 段（`enabled`/`refresh_service`/`refresh_timeout_s`/`max_corrections`/
    `position_tolerance_m`/`orientation_tolerance_rad`/`correction_settle_s`，**默认关闭 = 旧行为不变**）；
    `PickScene` 末尾追加可选 `arm_to_target`（默认 None，保持位置参数兼容）。
  - **观测/适配**：`PickObservationCache` 读取 `arm_base → grasp_target` 动态 TF（freshness/age 与
    object 同口径，refresh 后要求新样本）；`Ros2Transport.refresh_object_pose()` 调
    `/vision/refresh_object_pose`（`std_srvs/Trigger`），失败/超时明确 `ContractError`（不假装成功）。
  - **规划**：配置 `frames.grasp_target` 后 `planning._plan_from_target()` 直接取该 TCP 位姿作为
    grasp（仍走顶抓姿态 +Y 水平/+X 朝下、workspace、宽度校验），`pregrasp = grasp + 根系 +Z ×
    pregrasp_clearance_m`，observation 沿用 `_observation_pose`；未配置时行为与现状完全一致。
  - **执行**：新增 `vision_align`（观察位）与 `vision_pregrasp_align`（预抓取位）阶段（辅助抽到
    `stations/pick_place/vision_servo.py`）：refresh → 等新 TF 样本 → 重读 target → 超差才重规划；
    预抓取位最多 `max_corrections` 次校正；事件 `vision_refresh` / `vision_correction` /
    `vision_aligned`；达上限仍超差 → 整轮 `vision_alignment_incomplete` **明确失败**（不下探、不夹爪、
    不新增恢复动作），失败路径保持现有清理语义。
  - **配置模板**：`configs/pick_place/live_ros2_vision.yaml`（真机视觉目标 + 有界伺服）。
  - **测试**：新增 `tests/unit/test_pick_vision_servo.py`（未配置路径不变 / 校正收敛 / 超限失败 /
    refresh 服务失败 / grasp_target TF 缺失 / 事件字段），复用 `tests/unit/pick_place_support.py`；
    `pytest tests/unit tests/smoke` → **296 passed**（基线 259，+37；2026-11-10 integrator 复跑），域 231 ROS 回环全绿；mock 全链路视觉版
    `station/integration/run_vision_mock_chain.sh --rounds 3`（域 42）3/3 completed + 视觉事件。
