# AGENTS.md · rule_base_ros2（L3–L5 · I2+ 六边形子仓库隔离规则）

> 规则层级：用户当次明确要求 ＞ 本文件 ＞ [station 仓库规则](../AGENTS.md) ＞ [根 AGENTS.md](../../AGENTS.md)（工作区通用规则唯一权威）。本文件只写**本子仓库隔离边界 + 仓库专属硬约束 + 本仓库文档入口**，不复述上层文件：真机安全红线见根 §0、1000 行规则见根 §1、隔离级别定义见根 §2、修改纪律见根 §3；项目总体描述只在 [根 README](../../README.md)，开发规范以 [README](README.md)/[docs/ros2-adapter.md](docs/ros2-adapter.md)/[docs/develop.md](docs/develop.md) 为准。
> 隔离级别：**I2 分层隔离**（+ 六边形依赖更严，见 §2；定义见根 §2.1，分配见根 §2.3；station 子包口径见 [../AGENTS.md](../AGENTS.md)）。
> 文档入口：[README](README.md) · [适配契约](docs/ros2-adapter.md) · [develop](docs/develop.md) · [CHANGELOG](CHANGELOG.md) · [station api](../api.md) · [根 api](../../api.md)

## 1. 本子仓库在 I2 下的定位与附加边界

- **负责**：SO-101 数采工站的业务内核（Pick 站：顶抓规划/执行/观察位姿/每轮起止归零/完整 place）+ L3 ROS 2 适配器（`infrastructure/ros2/`）。公司 `rule_base` 移植版：已删 CORA，**不使用 CORA**，不重新引入原生 wheel / CORA manifest / CORA 专属 topic。
- **绝不负责**：环境 reset、初始位姿、物体摆放与业务结果判定（上游负责）；相机识别；L6 采集/质检/策略与 L7 CLI；不碰 L2 ros2_control/MoveIt，不改 `ros2_ws/src/**`（I0 上游只经 `so101-ros-physical-ai/patches/` 重放）。
- **对外只提供**：`python3 -m rule_base.interfaces.node` CLI + JSONL 事件流 + 稳定 Python API（见 [README](README.md)、[station api](../api.md) §4/§6）；ROS 侧只经 `infrastructure/ros2/` 消费 L1 `/arm/*` 与 TF。
- **开发约定**：Python 遵循 `pyproject.toml`（Ruff 行宽 100、`requires-python >=3.10,<3.14`、mypy strict）；新增/修改生产函数补类型注解；只补与行为有关的回归测试。

## 2. 六边形分层与依赖方向（唯一例外已给定，勿扩大）

- 允许方向：`interfaces/` → `application/` → `stations/`；依赖只指向更内层（`domain/` 的 Protocol/模型）与横切工具（`config/`、`observability/`、`infrastructure/transform/` 的纯数学/TF）；`infrastructure/ros2/` 实现 `domain/models.py` 的 Protocol，**不反向依赖** station 逻辑。
- **唯一例外**：`application/runner.py` 的 `serve()` 是**装配入口**，延迟 import ROS/假实现；`stations/pick_place/{execution,planning}.py` 只引用 `infrastructure/transform/math.py` 的**纯数学工具**（无 ROS）。禁止业务逻辑 import `infrastructure/ros2` 或直接调 ROS。
- 入口与生命周期在 `interfaces/`、`application/`，动作在 `stations/`，ROS 2 适配在 `infrastructure/ros2/`；事件只增不改（`observability/events.py`），新增字段保持 JSON 可序列化。
- **1000 行**：本子仓库继承根 §1；本仓库口径：**含 `tests/`**（测试同样计入行数）、800 行预警、超限先拆再提交；拆分后跑 §5 单测/冒烟并全绿，不得改行为/删测试凑绿。

## 3. 隔离契约（本仓库取值；通用条款见根 §2.2）

| 维度 | 本仓库取值 |
|---|---|
| 写入范围 | 仅 `station/rule_base_ros2/`（`src/`、`configs/`、`tests/`、`docs/`）；子包目录不越界 |
| 跨仓库读/接口 | 经 `infrastructure/ros2/` 调 L1 `/arm/*` + TF；消费 L0 `arm_msgs`；不 import 兄弟仓库源码；供 `station/integration` 编排调用 |
| 运行时 domain | 离线 fake 无 domain；ROS 回环/mock 用 **229**（contracts §8 S1 登记域）；真机 **0** 只在授权窗口（见根 §0）；Fast DDS ≤232 |
| 环境 | 系统 python（不建 venv；PEP 668：`pip install --user --break-system-packages --no-deps -e .`） |
| 数据/产物 | 不写数据面；日志/证据由调用方收集；checkpoint、`data/`、`policies/` 不入库 |
| git | 随 `station/` git 仓库（无独立仓库）；只操作本子仓库；不改/不暂存用户 WIP；**Lead 统一提交** |

- ROS 回环用隔离 domain **229**（contracts §8 S1 登记域；[docs/develop.md](docs/develop.md) §4）；同一 domain 不重复发布相同 child frame；残留 `/dev/shm/fastrtps_*` 先确认无存活节点再清（[station develop](../develop.md) §7.3）。

## 4. 不可放宽的硬约束（仓库专属，只能更严）

- **Pick 语义冻结**：保持 Servo、夹持、清理与**有界恢复例外**语义；`completed` 只表示动作及清理完成；不擅自增加失败后松爪、撤退或重试。
- **每轮起止归零（D-094）**：`home_start`/`home_end` 经 `ros2.home_joint_positions` 关节流式回折叠 rest（第 6 位夹爪用实测角保持）；失败直接整轮失败，**失败路径不新增失败归零**；`ros2.recover_to_home` 仍默认关闭（仅真机恢复可选）。
- **物体/放置位姿只读 TF**：`frames.object`/可选 `frames.place` 由上游 `tools/object_pose_publisher.py` 动态 TF 提供，本仓库配置**不写坐标**；`frames.place` 缺省时退化为 legacy（无 place 阶段）行为。
- **单位与帧**：位姿/夹爪位置用 m、关节用 rad、四元数 xyzw；`ros2.base_frame` 必须等于 `frames.arm_base`（启动即校验）；真机 `tcp_frame` 为 `grasp_tcp_link`；`acquire_control: false` 时不调用 Arm 服务。
- **超时与失败**：超时用单调时钟、数据时效沿用源时间戳、等待可中断；失败不假装成功（不可达抛 `ContractError`），不放宽容差/延长超时/关闭校验。
- **真机操作按根 §0**：仅用户在场 + Lead 授权；本仓库默认只跑离线/mock，真机 `--run` 只在授权窗口执行；离线/mock 通过**不代表**真机通过。

## 5. 验证入口（改什么 → 必跑什么 + 文档出处；环境/命令以文档为准）

- 环境：系统 python（不建 venv）；`export ROS_DOMAIN_ID=229` → `source /home/ubuntu/yayyrobotics/ros2_ws/setup_ros_env.sh`；本机 editable install 指向旧副本，运行/测试前置 `PYTHONPATH=<repo>/src:$PYTHONPATH`（[README](README.md) §环境）。
- 行为改动：`python3 -m pytest tests/unit tests/smoke -q`（离线单测 + 冒烟，见 [README](README.md) §测试）。
- ROS 适配/装配改动：另跑 `ROS_DOMAIN_ID=229 python3 -m pytest tests/integration/test_ros2_loopback.py -q`；必要时 `bash ../integration/run_mock_chain.sh --rounds 10`（mock 全链路，[station develop](../develop.md) §2.1）。
- 1000 行拆分后：同上单测/冒烟 + 涉 L3 时跑 mock 全链路（根 §1、本文件 §2）。
- 真机 `--run`：只在根 §0 授权窗口执行，不重复索要会话内授权；交付简述改了什么、跑了什么、未验证范围。

## 6. 交付前自查

- 只改本子仓库；不顺手改 `station/` 其他子包或兄弟仓库（跨仓库按根 §2.4）；不碰 `ros2_ws/src/**`、`leLab/`。
- 本文件只写本子仓库边界，不复述根 §0/§1/§2/§3；发现与根冲突按根执行并修正本文件（根 §4「规则一致性」）。
- 改 Markdown 链接后跑 `cd /home/ubuntu/discover && python3 tools/check_docs_links.py`（断链 0）；不执行 `git add/commit/push`（**Lead 统一提交**）。

## 附：上层规则与文档索引

- 上层规则：[根 AGENTS.md](../../AGENTS.md)（§0 安全红线 · §1 单文件 ≤1000 行 · §2 隔离级别 I0–I3 · §3 修改纪律 · §4 只读检查）· [station AGENTS.md](../AGENTS.md)（本子仓库的父仓库规则）。
- 本仓库文档：[README](README.md) · [适配契约 docs/ros2-adapter.md](docs/ros2-adapter.md) · [开发说明 docs/develop.md](docs/develop.md) · [CHANGELOG](CHANGELOG.md)；跨仓库接口见 [根 api.md](../../api.md)、[station api.md](../api.md)。
