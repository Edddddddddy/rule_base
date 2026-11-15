# Changelog

## Unreleased

- **2026-11-15 · 真机/推理服务交付默认切到 env 位置订阅 + legacy 死文件清理（用户确认「最小改动 C」）**：
  - **默认交付**：`submission.yaml` 的 `runtime.uv.argv` `--config` 由
    `configs/pick_place/live_ros2_vision_instance.yaml` 改为
    **`configs/pick_place/live_ros2_env.yaml`**（env 位置订阅：物体 id+坐标来自 env_app 动态
    `/tf` `env_marker_<id>`，放置坐标来自 `/tf_static` `place_target_01`，每轮观察位后取**一次**
    TF 快照整轮复用）；`arm.manage_lease: false`（租约由 factory_manager 持有，实例不
    acquire/release）、`arm.acquire_control: true`。`live_ros2_teach.yaml` /
    `live_ros2_vision_instance.yaml` 仅保留为**非默认回退**（不删除配置/代码）；submission
    头部注释同步。
  - **删除 legacy 死文件**：`tests/integration/config_real_backend_mock.yaml`（先以
    `grep -rn` 复核 `.py/.md/.yaml/.sh/.toml` 全仓零引用——仅历史 CHANGELOG 提及；文件自述
    “historical reference only”，WP5.2b 调查遗留）。未发现其他可安全删除的零引用死文件
    （`ros2_mock_arm.py`、`instance_e2e_support.py`、`instance_entry_mock.py`、
    `config_ros2_loopback.yaml`、`test_ros2_loopback.py` 均被 integration 测试/文档引用 → 保留）。
  - **契约测试**：`tests/smoke/test_repository_contract.py` 新增
    `test_submission_defaults_to_env_position_subscription`（解析 submission argv → `--config`
    指向 `configs/pick_place/live_ros2_env.yaml` 且相对路径存在、可 `load_settings`，断言
    `env_frames.enabled is True`、`arm.manage_lease is False`、`arm.acquire_control is True`）+
    `test_legacy_backend_mock_config_stays_removed`（防回潮）；既有配置清单/断言全部保留。
  - **文档**：`README.md`（常用命令/实例模式/env 段：env 为真机与推理服务交付默认并给默认命令，
    teach/vision 标注非默认回退）、`docs/instance-mode.md` §2（默认入口命令改为
    `live_ros2_env.yaml`）。
  - **HEAD 遗留漂移修复（单列，非本任务引入）**：`tests/unit/test_pick_env_frames.py` 硬编码期望
    仍是旧标定 `(0.0014, 0.0090, -0.0114)` → 同步 HEAD 437ece0 的 yaml 值
    `(0.0039, 0.0177, -0.0147)`（断言强度不变）；`docs/ros2-adapter.md` §9.1/§9.2 旧
    `snapshot_timeout_s: 2.0` 与旧偏置 → 当前 yaml 值（事实对齐）。
  - **验证（离线）**：`PYTHONPATH=$PWD/src:$PYTHONPATH /usr/bin/python3 -m pytest tests/unit
    tests/smoke -q` → **520 passed**（HEAD 基线实测 1 failed/517 passed，修复漂移后全绿）；
    `ruff check src tests` → 与基线一致 6 个既有 I001/F841（均不在本次改动文件，无新增）；
    Python 校验脚本直接加载 submission argv 配置：`env_frames.enabled=True`、
    `arm.manage_lease=False`、`arm.acquire_control=True`。
  - **边界/未验证**：真机 env 模式复验由 **Lead 在本轮真机窗口执行**（本 commit 只含离线/契约
    证据，**未 push**；推送需 Lead 真机验收通过）；不改业务代码语义、wire/契约、其他配置、
    其他仓库；env 模式已知几何精度限制见上一条 2026-11-14 条目。

- **2026-11-14 · 新增 env 位置订阅模式（第三种坐标来源，additive；用户现场架构要求）**：
  - **背景**：用户要求不使用固定静态示教、不使用腕部检测，物体 id+坐标改由 env_app 发布的
    动态 TF `env_marker_<target_object_id>` 提供，放置坐标由其 `/tf_static` 的 `place_target_01`
    提供；**观察位到位后读一次快照，整轮复用**（不逐帧跟随）。
  - **配置**：`configs/pick_place/live_ros2_env.yaml`（新增；不改已真机验收的
    `live_ros2_teach.yaml`/`real_pick.yaml`/`submission.yaml`/`teach_points.yaml`）；
    `PickSettings` 新增 `env_frames:` 段（`enabled/object_tf_prefix/target_object_id/
    place_frame/snapshot_timeout_s/observe_*/observe_settle_s`），默认关闭；`target_object_id`
    支持 `RULE_BASE_TARGET_OBJECT_ID` 环境变量覆盖（空值 → 配置错误；非 env 配置忽略）。
    配置校验：env 模式禁止同时配置 `vision.enabled`/`frames.object`/`frames.grasp_target`/
    `frames.place`；非 env 模式 `frames.object` 仍必填；`PickFrames.object` 变为可选。
  - **执行**：`stations/pick_place/env_snapshot.py`（新增）：home_start/initialize_servo 后到观察位
    （`observe_joint_positions` 关节空间，缺省 TCP 观察位）→ settle → 有界重试取一次快照
    （`snapshot_timeout_s`）→ `pick_plan`；TF 缺失/超时 → `env snapshot timed out ...` 整轮失败
    （stage `env_snapshot`，不进入抓取恢复、不夹爪）。`PickRound` 在 env 模式下把规划推迟到
    观察位之后；warmup 只等 `root→arm_base`（`InstanceRunner.robot_only_warmup`，保留
    `vision_observe_first` 兼容别名）。
  - **几何**：grasp TCP = marker 位置 + `profile.grasp_contact_offset_tcp_m`（TCP 系按顶抓参考
    旋转合成），姿态恒为 `reference_tcp_orientation_root_xyzw`（D-047）；pregrasp/observation/
    post_grasp/place approach/retreat 沿用 `real_pick.yaml`；env 模式宽度区间直接用 profile
    `cube_side_m`/`grasp_min/max_width_m`（marker 旋转不当作立方体轴，实测闭合宽度仍决定成败）。
  - **TF 缓存**：`PickObservationCache` 改用有效帧（env 模式 object=`env_marker_<id>`、
    place=env `place_frame`、不要求 grasp_target；缺失时报
    `Pick object/place TF is missing: base_link <- ...`）；`FakePickEnvironment` 同步支持 env
    模式（marker 动态 `/tf` + place 一次性 `/tf_static`）。
  - **现场标定偏置 `env_frames.grasp_offset_arm_m`（2026-11-14 真机首轮反馈）**：env 只发码面
    位姿，首轮真机因「TCP = 码面位置」在方块顶部上方闭合而空夹（close 宽度 0.0047 m）。新增
    arm 系固定抓取偏置（`|每轴|<=0.1 m`，配置校验 fail-closed）：grasp TCP = marker 位置 +
    `profile.grasp_contact_offset_tcp_m` + `grasp_offset_arm_m`；方法 = 手拖正确抓取 TCP −
    同时刻 env_marker 位姿。现场两轮：`[-0.0008, 0.0086, -0.0093]`（方块 x≈0.23）与最终
    `[0.0014, 0.0090, -0.0114]`（方块移至 x≈0.28 可达区后；当前 `live_ros2_env.yaml` 值）。
    另：任务 marker 实际边长 2 cm（env site 配置 `marker_size_m: 0.020`，与 discover
    `objects_aruco_2cm` 一致；按 18 mm 估会把位姿整体缩向相机约 2.7 cm）。
  - **2026-11-14 真机验证（env 模式，domain 0）**：链路已验证——env 动态 `/tf`
    `base_link→env_marker_1`（~15–20 Hz）+ env `/tf_static` `base_link→place_target_01`
    （[0.229637, 0.149113, 0.008321]）→ 观察位 `env_observe_joints` → 单次 `env_snapshot`
    （`env_marker_1`/`place_target_01`）→ `pick_plan` → 臂运动至规划抓取位 → 合爪/恢复/home
    全流程跑通、事件与失败语义符合契约（cleanup_confirmed=true，exit 0）。**抓取成功未达成**：
    4 次真机尝试均为空夹（合爪宽度≈0.005 m）；归因 = env 外参 `review_required`（14.3 mm/
    2.81°）导致的目标位姿精度不足 + 方块在观察/下探路径上被碰移（env 位姿快照前后变化 5–10 mm）
    + 抓取深度需按物体逐个标定；下一步需提升 env 外参标定精度或改「物体中心+夹持深度」由上游
    任务规格给出后再复验。真机证据见 workspace `.scratch/real-verification/evidence/
    env_rule_base_integration/`。
  - **D-142**：`validate_live_grasp_source()` 允许 env 模式作为 live 位置来源（与实时视觉
    `frames.grasp_target` 并列）；仅静态 `frames.object` 的配置仍被 `--run` 拒绝。
  - **测试**：新增 `tests/unit/test_pick_env_frames.py`（32 项：配置/环境变量覆盖/非法 id/
    互斥校验/快照几何（marker 位置+contact offset+根臂帧换算+忽略 marker 旋转）/tf 缓存快照/
    单快照轮次/快照重试/快照超时/warmup 门），smoke 配置清单加入新配置。
    `pytest tests/unit tests/smoke -q` → **516 passed**（基线 484 + 32）；现有 teach/vision/mock
    路径零回归（分支 additive、默认关闭）。真机未验证（需 env_app 与现场窗口）。
- **2026-11-13 · v0.7 tools/ 废除：mock 夹具消费路径 env 化 + 实例 E2E 对齐 contracts R14（仅测试/文档，业务代码零改动）**：
  - `tests/integration/instance_e2e_support.py`：`FAKE_MANAGER` 改为 `MOCK_FIXTURES_DIR`（默认
    `<ws>/factory_manager/local/integration/mocks`；缺失时 `pytest.skip` 并给修复命令），不再写死顶层 `tools/mocks`；
    `test_instance_entry.py` 的接口覆盖层改 source `<ws>/inference_manager/outputs/ros2-interfaces/install/setup.bash`
    （`inference_manager_msgs` 唯一 owner）。
  - **修既有测试漂移**：S1 实例 E2E 在 contracts v1.2 **R14**（`manage_lease:false` 轮末保持 SERVO(3)、由 FM 统一
    `switch_idle`）落地后仍按"轮末 3→0"断言，迁移前即红 → 测试改为每轮只切 `IDLE(0)` 作为租约保持证据（下一轮 entry
    子进程自行 0→3）；**断言强度不变**，只移除 R14 下非法的 3→3 调用。`docs/instance-mode.md`、`README.md` 同步。
  - 验证：`tests/unit tests/smoke` → 484 passed；实例 E2E（domain 229）→ 1 passed；真机未验证。
- **2026-11-13 · S5 集成小修：`arm.manage_lease:false` 时轮末不再 `switch_state(0)`（contracts v1.2 R14，mock 全栈证据）**：
  - **问题**：S5 全栈 mock E2E 实测，实例配置 `manage_lease:false`（租约/状态机由 factory_manager 持有）时，
    每轮结束 `PickRound` 清理调用 `arm.stop()`，原逻辑在 `acquire_control:true` 下会 `switch_state(0)`；
    FM 运行期以 `/arm/state=SERVO(3)` 监控租约，轮末被切 IDLE 会误判 `AUTHORITY_LOST(2003)` 并终止任务。
  - **修复**：`src/rule_base/infrastructure/ros2/arm_topic.py::stop()` 增加 `manage_lease:false` 早退分支：
    只清本地会话标志、**不切 IDLE**（保持 SERVO(3)），由 FM 在任务结束/失败清理时统一
    `switch_idle → release`（contracts §3 停止顺序）；`manage_lease:true`（独立运行）行为不变。
  - **验证**：S5 全栈 mock（domain 198，arm_app mock 后端 + 真实 S2/S3/S4）任务 3 轮 `COMPLETED`、
    cleanup 8 步全 success、无残留；`tests/unit` 全量回归通过。真机未验证（真机 Gate 由 S5 另行授权执行）。
  - **边界**：不改 wire/接口/业务内核其他逻辑；不影响 `manage_lease:true` 的单机运行语义。
- **2026-11-13 · 新增 026 实例入口（S1 实例化改造 RB-P1/P2，mock/本机证据）**：
  - **背景**：inference manager 需要按 026 协议拉起算法程序；原仓库只有站侧 CLI。业务内核
    （`application/stations/infrastructure/config`）**零改动**，入口每轮调用原 CLI 跑一轮。
  - **新增 `src/rule_base/instance/`**：
    - `protocol.py`（纯逻辑，无 ROS）：`INFERENCE_NAMESPACE/DOMAIN_ID/STATION_TYPE/CLIENT_ID/
      LEASE_ID/INFERENCE_ID` 解析与校验（DOMAIN_ID 必填、与 ROS_DOMAIN_ID 不一致/STATION_TYPE 非
      `pick_place`/非法 namespace → 配置错误）、namespace→话题、**False→True 边沿状态机**
      （先见 False 武装、True 消费一次、运行期 reset 丢弃、轮后持续 True 不重复开轮）、退出码归类；
    - `keepalive.py`（纯逻辑，无 ROS）：hold 命令=最近实测 6 关节位置（夹爪保持）、关节顺序按配置、
      残缺/非有限样本跳过、2–5 Hz 频率校验（默认 4 Hz）、start/stop；
    - `runner.py`：单轮子进程冻结 argv（live=`--rounds 1 --run`；mock=`--dry-run --transport ros2`）、
      继承 env 并把**本仓库 `src` 前置 PYTHONPATH**（避免本机 editable install 的旧副本）、cwd=仓库根、
      单轮超时 600 s（SIGTERM→5 s→SIGKILL）、退出码归类（0/非 0/中断/spawn 失败）；
    - `entry.py`（rclpy 延迟 import）：env/config → 派生 ROS_DOMAIN_ID → rclpy.init → warmup（等
      `<prefix>/joint_states` 首样本，默认 30 s，失败退出 2）→ `stopped=True` 2 Hz（RELIABLE+
      TRANSIENT_LOCAL+depth1）→ 边沿开轮（先 False→子进程→True）→ SIGTERM 清理退出 0；
      单轮失败输出 `stage`/`error`/`inference_id` 结构化 JSON → `stopped=True`（留 ≥1 周期）→ 退出 1；
      stdout 全为单行 JSON 事件（`instance_starting/instance_ready/instance_topics/round_started/
      round_finished/instance_failed/instance_stopping`）。
  - **边界（冻结）**：实例不调用 `AcquireAuthority`/`ReleaseAuthority`（`manage_lease:false` 由实例
    配置保证，rule_base 仅 `SwitchState(3)`）；不写 `/follower/forward_controller/commands`；子进程
    运行期间严格停发 keepalive；不启真机（`--dry-run` 仅测试）。
  - **测试**：新增 4 个离线单测文件 `tests/unit/test_instance_{protocol,keepalive,runner,entry}.py`
    （**105 passed**，无 ROS 亦可跑；含边沿语义/env/退出码/argv/PYTHONPATH/SIGTERM 转发/超时强杀/
    spawn 失败/CLI 失败路径）；新增集成测 `tests/integration/test_instance_entry.py` +
    `instance_e2e_support.py` + 包装脚本 `instance_entry_mock.py`：用 `tools/mocks` 的
    `fake_inference_manager.py --instance-script` 拉起**真实 entry**，在 ROS domain **229**（contracts §8）
    上跑 mock arm（`tests/integration/ros2_mock_arm.py`）+ 测试扮演 FM 先 acquire 租约：验证 ready/stopped
    时序、3 轮 reset 边沿、每轮结束后空闲 >5 s 再 `switch_state(3)→(0)` 均 success（keepalive 续租证据）、
    空闲 hold ~4 Hz 且位置=实测 joint_states、轮后持续 True 2 s 不重开轮、stop 后 entry exit 0、manager
    SIGTERM exit 0、无残留进程。domain 229 与 station_support SUP-P1 集成测共用（contracts §8），
    集成测内含「域静谧预检」（foreign TF 父帧/多余 /arm 发布者/foreign 节点），检测到并发会话时等待并明确
    提示串行执行（不是契约变更）。
  - **配置/文档**：新增 `docs/instance-mode.md`（入口/环境变量/话题 QoS/状态机/keepalive/runner/退出码/
    日志/安全边界/mock E2E 命令/已知限制）；本 README 增加「实例模式（026）」与测试命令。实例配置模板
    （`configs/pick_place/{mock_ros2_instance,live_ros2_vision_instance,live_ros2_teach}.yaml`）、
    `submission.yaml`、`uv.lock` 由 Lead 统一维护（S1 任务板 task-3），不在本条改动内。
  - **证据分档**：全部为 mock/本机证据；未启真机、未使能运动、未 push。
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
