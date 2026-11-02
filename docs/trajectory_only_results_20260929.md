# 2026-09-29 纯轨迹离线优化结果

## 17:31 完整清单与起落架补传

此前遗漏的起落架有界上/下行及配套周期/完整动作/库已补传，与推进3mm和最新pull一起保存在 Jetson `trajectory_candidates/session-bounded-20260929T093102Z`；18个JSON文件哈希核验通过，未激活，旧版保留。快速逆放没有被自动选中。已实算逆放与上杆停稳、上杆退出、准备位接近以及下杆退出的边界，均能按正确路由C2衔接，但中段接触安全仍未确认。详见本会话完整清单 `docs/session_trajectory_inventory_20260929.md` 和 `data/rail_deployment/session_inventory_20260929/gear_connections.json`。

## 17:09 更新：推进 3 mm、拉出截尾 5 mm、抓握前去迟滞

用户要求保留旧版，推进选 3 mm；拉出少约 5 mm，可沿原路径截尾，并沿用当前速度策略。随后追加检查拉出接近末段、切换 `pull_grip` 前的低速停滞。

### 拉出少 5 mm

- 以本日有界优化 pull 为基线，保留已优化的 7.758369332 s 退出段。
- 拉出沿 root 负 X，故少拉出 5 mm 对应松手前指尖 X 从 **416.739032 到 421.739032 mm**。
- 在原接触路径时钟 **3.682504931 s** 截尾，删除原末尾约 0.633014 s；截断前原路径 0.2 s 通过单调 C2 时间映射变成 0.4 s 平滑制动，停稳后松手。不是直接删除非零速度终点。
- 接触单元 **4.315518901 → 3.882504931 s**；保留路径的时钟插值误差上界约 6.69e-9 rad，未全局平移。
- 沿原路径提前松手带来 Y **-2.846930 mm**、Z **-0.995691 mm**、腕/末端姿态约 **0.848312 度**变化，必须明确：这是用户允许的截尾实现，不是仅 X 平移的 IK 方案。
- 松手期间机械臂保持新的截尾端点，手型、手指参数、顺序与原等待时长不改。退出前 **0.75 s** 使用 C2 连接接回原有界优化路径，之后 q/v/a 保持原样；早期退出采样 TCP 差约 5.890243 mm，属于新松手位置所需的几何衔接，不冒充原路径 5 mm 平滑预算。
- 连续限值及完整组合 C2 通过；接触/退出均未增加内部停顿，退出未引入可见 X 反向。接触 jerk 4.964030，退出 jerk 4.937274 rad/s³，未放宽限值。
- 报告：`data/rail_deployment/pull_shorter_5mm_20260929/report.json`。该中间候选已独立上传并保留：远端 `trajectory_candidates/push3-pullshort5-20260929T085654Z`。

### 抓握前低速迟滞

确认问题在 `motion_0` 接近段，而不是 `hand_1` 的手型等待。原接近末段仍有毫米级漂移和停顿；本次仅对有剩余预算、互不重叠的两段做 C2 替换：原时钟 **16.0–19.5 s → 0.50 s**、**19.5–22.00715012 s → 0.50 s**。16 s 前 q/v/a 不变，最终抓握 q/v/a 不变，后续手型等待、5 mm 截尾接触段和退出段都不变。

| 指标 | 修改前 | 修改后 |
| --- | ---: | ---: |
| 接近段时长 / s | 22.007150 | 17.000000 |
| 接近 TCP <5 mm/s 时间 / s | 10.138687 | 4.570000 |
| 接近内部停顿数 | 4 | 2 |
| 完整拉出预览，含原手型等待 / s | 36.648024（已截尾版） | 31.640874 |

相较已部署完整拉出预览 39.023583 s，新组合约省 7.382709 s，其中约 1.942545 s 来自此前退出优化、0.433014 s 来自截尾、5.007150 s 来自本次接近去迟滞。仍未声称消除所有低速段。

两段分别对照冻结原路径误差台账计算剩余预算，无重叠、无重复消费；累计 TCP 保守上界最大 **4.787512 mm**，保持既有非接触 5 mm、joint 0.015 rad、姿态 2.5 度预算。接近连续速度/加速度/jerk 峰值仍为 **0.394145 / 0.791659 / 4.964031**，腰部与 100 Hz 单步限制均不改。切换 `pull_grip` 后的 1 s 手型等待保留，绝不是提前闭手绕过等待。

报告：`data/rail_deployment/pull_ready_shorter5_20260929/report.json`。最终 pull 快照：`data/rail_deployment/pull_ready_shorter5_20260929/individual/ae2d838b8a3b866b234d9ea5c9872f571327cbdba454cdbcd60c307d38a3e21f.json`，选择 `flow`。

### 面板与上传状态

- 标准离线面板“优化对比 → 滑轨：完整拉出”新增两个组：“拉出少 5 mm / 松手位置对比”和“拉出接近去迟滞 + 少拉出 5 mm”。前者聚焦松手前位置，后者聚焦抓握前位置；均提供无箭头轨迹及原速度/导数视图，不覆盖旧有界组或推进补深组。
- 最终候选组合指定 **push = push-depth-exit-v2-3mm**，**pull = pull-ready-c2-shorter-x5mm-v2**；不修改起落架或拉环，不部署任何运行时代码。推进补深版未合并独立的 0.19 s 退出提速候选，但沿用相同速度/加速度/jerk 策略。
- 2026-09-29 17:09 CST 上传至 `/home/unitree/rh56dfx/workspace/execution/trajectory_candidates/push3-pullshort5-ready-20260929T090936Z`，13 个 JSON 文件逐项 SHA256 核验通过。包含在线旧三份配置、冻结旧 push/pull、原有界优化 pull 和仅截尾 pull 快照，旧目录也均保留。
- 回执：`data/rail_deployment/push3_pullshort5_ready_upload_20260929T090936Z/upload_receipt.json`。工具：`scripts/stage_push_depth_3mm.py --include-pull-ready`，仅暂存上传，不提供安装或激活操作。
- **上传不等于激活。** 复查时 Jetson 正在执行任务，`controllerRunning=true`、`phase=RUNNING`、`controlOwner=jetson`。在线三份配置哈希和已加载版本 `gear-bounded-v6-330f290121485a5e` 未改变，未重启服务、未发送任何实机命令。正式切换须在重新确认安全失能和新几何的现场可行性之后另行进行。
- 截尾测试 4 项、接近优化测试 4 项、空间轨迹测试 3 项、快照测试 4 项，共 15 项通过。覆盖完整组合连续限值、截尾路径保持、新松手静止段、抓握位姿不变、非重叠路径预算、手型事件相对时间、标准面板恢复与显示。截图位于上述两个报告目录。
- 随后既有优化器完整回归 12 项也通过，本轮最终合计 27 项通过。本机离线面板服务已重载以显示新增比较组；未操作 Jetson 实机服务。

## 16:36 上传记录：3 mm 待审核候选

按用户要求已将 3 mm 候选上传 Jetson，但未激活、未替换在线配置。远端目录：`/home/unitree/rh56dfx/workspace/execution/trajectory_candidates/push-depth-3mm-20260929T083633Z`。目录仅含冻结快照、校验证据和 manifest，明确选择 `flow` 即 `push-depth-exit-v2-3mm`；不是可安装发布包，保留 `hardware_executable=false` 和实物检查待完成标记。

- 上传前重新加载冻结模型和快照，连续导数校验通过；上传后逐文件 SHA256 核验通过。
- 在线 `selected_bundle.json`、`gear_protocol.json`、`dring_protocol.json` 字节哈希未改变，已加载版本仍为 `gear-bounded-v6-330f290121485a5e`。
- 只读健康核查：无活动任务，`controllerRunning=false`、`controlOwner=none`、`phase=DETACHED`；同时有 `requiresRecovery=true` 和 `Unexpected controller/diagnostic publisher count` 提示，不能把此状态宣称为全部健康检查通过。
- 未重启任何 Jetson 服务、未初始化、未发送运动或手型命令；不修改任何生产运行时代码。
- 激活前仍需确认现场安全失能、补深轴线及机械余量，核清健康提示，并基于届时在线父版本构建和校验仅修改 push 的发布包。当前上传未混入 push 退出提速、pull 或起落架的新优化。
- 本地回执：`data/rail_deployment/push_depth_3mm_upload_20260929T083633Z/upload_receipt.json`；工具：`scripts/stage_push_depth_3mm.py`。

下文“尚未上传/未入包”等为此前阶段记录；本次只完成候选暂存上传，没有改变未激活状态。

## 后续更新：标准面板毫米级轨迹对比

用户随后明确允许推进后的退出路径略有变化。新增比较组不替换下文的历史报告或 `trajectory_only_20260929/data_only/` 数据包。

- 入口仍为 `bash /home/syf/g1_upper_body_ws/scripts/start_task_panel.sh --offline`。
- 选择“优化对比 → 滑轨：完整推进 → 对比组：推进补深 0 / 3 / 4 mm → 末端轨迹”。新启动默认恢复此冻结比较，不需生成。
- 灰蓝为冻结已部署基线 0 mm，橙色为 3 mm，绿色为 4 mm。三者同时叠加，仅线条、没有箭头；可单独隐藏、切换 XZ/XY/YZ、滚轮缩放、拖动平移，以及切换全路径/最深点局部视图。
- 局部默认横向视野 24 mm，显示采样相邻距离不超过 0.05 mm、中点弦偏差不超过 0.005 mm。这是显示精度，不是物理精度或连续碰撞证明。
- “有界优化”组仍保留推拉和起落架的低速/停顿优化快照，也提供末端轨迹叠加；本次补深独立基于冻结已部署 push，没有声称已同时合并有界提速候选。

| 补深 | 最深模型 X / mm | 相对 0 mm 的 X 增量 / mm | 峰值后退出过渡 / s | 最大 jerk / rad/s³ |
| --- | ---: | ---: | ---: | ---: |
| 0 mm | 445.337704 | 0 | 无修改 | 4.978538 |
| 3 mm | 448.292573 | 2.954869 | 1.50 | 4.992285 |
| 4 mm | 449.277529 | 3.939825 | 1.75 | 4.978538 |

三者连续导数检查通过，原速度/加速度/jerk 策略未提高。4 mm 的 1.50 s 过渡仍失败（jerk 5.019715），延长到 1.75 s 后通过；运动总时长不变，29.105021 s，含原手型等待的完整预览为 31.105021 s。

旧结论“严格保留退出的 3/4 mm 候选 jerk 超限”表示：推进更深后，必须在很短时间内回到完全不变的退出路径，导致加速度变化率超过当前软件策略。允许早期退出路径变化可把补偿平滑撤回。这里的 1.50/1.75 s 是最深点之后的过渡时窗，不是额外等待或整段退出时长。

补偿沿记录估计轴线，不是实测水平滑轨轴，所以 3/4 mm 补偿对应 +X 约 2.955/3.940 mm。轴线、末端机械余量及实际接触安全仍待确认。“未入包”指未加入待发布轨迹数据包，更未部署到实机；不妨碍离线面板观察。用户对退出几何变化的许可不等于实机执行授权。

新增报告：`data/rail_deployment/push_depth_exit_v2_20260929/report.json`。冻结比较：`data/offline_optimizer_previews/f5f832f2781c093b582d6a3fbe5e3e52b6405b62afc1f8d6ab8852bc16dddfb7.json`，通过 `experiments.json` 注册，不替换有界 catalog。

新增 `test_task_spatial_comparison.py` 的 3 项检查通过，覆盖连续导数、线条类型/采样精度、标准面板恢复/切换/播放，以及标准和窄窗口布局。截图：同报告目录下 `standard_panel_depth.png`、`narrow_panel_depth.png`、`depth_xz.png`、`depth_xy.png`。本机离线服务已重启，硬件网关与执行输出关闭；真实桌面截图接口返回黑图，显示验收以离屏 Qt 截图为准，不声称已完成实时 RViz 目视验收。

提供冻结 URDF 后，既有 `test_task_offline_optimizer.py` 最终完整重跑 12 项通过；此前一次运行曾出现 `right_shoulder_pitch_joint` 键缺失，单项复跑及完整复跑均通过，未为此修改轨迹生成业务逻辑。

## 状态与边界

- **没有部署，没有初始化，没有发送运动或手指命令，没有重启任何 Jetson 服务。**
- 开始时通过只读 SSH 核对配置、健康接口和文件哈希。线上仍为 `gear-bounded-v6-330f290121485a5e`，与交接凭据一致；核查时为 `controllerRunning=true`、`controlOwner=jetson`、`phase=ARMED_HOLD`，不满足更新条件。
- 当前运行时与本地历史版本有差异。本轮仅将 Jetson 当前 `gear_protocol.py`、`gear_progress.py` 等读入本地审计目录作校验，**没有把任何运行时代码放入数据包**。
- 只新增离线工具、测试、报告和快照，并更新 WSL 离线预览 catalog。没有修改协议、count、回调、恢复、控制权、手指顺序、增益或网关限值。
- 所有结果是命令轨迹的离线结果，不是实物任务成功或接触安全证明。

工作目录：`data/rail_deployment/trajectory_only_20260929/`。

## 有界候选

以下时长均为对应运动单元，不包含初始化、手指等待或真实控制器跟踪减速。

| 单元 | 已部署基线 / s | 新候选 / s | TCP <5 mm/s 时间，前→后 / s | 内部停顿，前→后 |
| --- | ---: | ---: | ---: | ---: |
| push 完整运动 | 29.105021 | 28.916047 | 3.8343→3.6245 | 6→6 |
| pull 接近 motion_0 | 22.007150 | 22.007150 | 10.1387→10.1387 | 4→4 |
| pull 接触 motion_1 | 4.315519 | 4.315519 | 0.7242→0.7242 | 1→1 |
| pull 退出 motion_2 | 9.700914 | 7.758369 | 2.4840→0.4199 | 1→0 |
| 起落架下→上 lu | 9.099693 | 8.963197 | 2.4749→2.3345 | 3→3 |
| 起落架上→下 ul | 16.354511 | 16.054511 | 6.5848→6.0898 | 2→1 |

pull 三段运动合计 **36.023583→34.081038 秒**；起落架 L→U→L 接触周期 **25.454204→25.017708 秒**。没有改变 GEAR_OPERATION 的业务周期定义。

离线面板完整快照还包含原有手型等待；例如 push 为 **30.916047 秒**，pull 为 **37.081038 秒**。起落架面板保持其原有组合选择，不把该预览的多行程组合误称为业务 count=1。

### 选择与拒绝

- 使用既有五次 C2 局部替换与速度重整，保持局部 q/v/a 边界；不是仅缩短时间戳。
- 从已有编辑记录把误差预算映射到已部署时钟，本次编辑只使用该区间的剩余预算。遇到二次平滑或不能追溯的回退结果，重新从冻结的原路径 baseline 生成，不能叠加一份完整预算。
- 接触仍为 TCP 1 mm、joint 0.003 rad、姿态 0.5 度；纯非接触段为五倍。新增编辑区间累计 TCP 上界分别约为：push 0.5724 mm、lu 0.2768 mm、ul 0.8758 mm、pull 退出最大 4.5194 mm。未修改区间沿用此前预算，不应把这些局部值当成整条轨迹的新总上界。
- pull 接触试验曾得到 4.282433 秒，但内部停顿从 1 增至 2，**拒绝选用**。
- push 本次有界修改位于原已部署时钟约 24.046–24.586 秒的退出局部，推进接触段和最深位置没有改变；它不是补深结果。
- pull 接近段没有找到满足约束且更好的候选，保留原样。低速并不全部意味着可删除：局部换向、接触衔接和约束内的腕运动不能强制拉直。
- 下行腕 roll 累计运动只从约 0.156842 降至 0.156677 rad；上行从约 0.006608 降至 0.006588 rad。没有声称已经消除大部分腕部往复。

### 连续导数检查

| 单元 | 最大速度 rad/s | 最大加速度 rad/s² | 最大 jerk rad/s³ |
| --- | ---: | ---: | ---: |
| push | 0.398049 | 0.894719 | 4.984849 |
| pull 接近 | 0.394145 | 0.791659 | 4.964031 |
| pull 接触 | 0.144577 | 0.458677 | 4.964030 |
| pull 退出 | 0.398049 | 0.916578 | 4.937274 |
| lu | 0.304123 | 1.045914 | 4.925746 |
| ul | 0.392468 | 0.838335 | 4.800003 |

速度与加速度限制来自冻结 action 的真实 `planning_config` 和 `motion_limits`，不是另写默认值。已校验连续多项式位置极值、速度/加速度/jerk 极值、腰部包络及 100 Hz 单步上界；腰部仍不超过 0.35 rad/s，单步不超过 0.006 rad。**5 rad/s³ 是现行软件策略，不是厂商认证的电机上限。**

详细每关节峰值、腕运动、TCP 路长、各坐标轴累计多余行程、低速时间、接受/拒绝理由见三个 `*_report.json`。TCP 运动指标按约 5 ms 采样，内部停顿为关节速度诊断；这些采样指标不替代连续限值校验。整条推拉包含必要往返，因此累计多余行程不能全部解释成异常抖动。

## 起落架逆放

复用原 lu 正确逆放的 9.099693 秒候选，对照当前已部署 ul 16.354511 秒：

- 端点一致，连续导数通过。
- 采样双向最近 TCP 路径距离仍约 **18.9844 mm**，近邻姿态差约 **18.7827 度**。
- 这是近邻诊断，不是严格接触对应，更不是碰撞或握持安全证明。
- 方向相关接触、锁止约束、握持容许姿态没有实物数据可确认，**不选入数据包**。

见 `reverse_report.json`、`reverse_panel/`。有界下行收益有限，不能用逆放的时长收益掩盖接触路径改变。

## 推进局部补深

从冻结 push 单段的接触起终点估计了一个**未实测的轴线假设**：URDF root 中 `[0.984956, -0.022443, 0.171341]`，与 root +X 夹角约 **9.9509 度**。它带有竖直分量，不能直接当成实际滑轨轴，尤其不能把“沿该轴 4 mm”说成“实际水平进深 4 mm”。

既有 `rail_accuracy_20260928/analysis.json` 表明原版与优化版最深 X 只差约 0.00876 mm；历史关节 FK 跟踪误差约 -0.1067 mm，均不足以解释用户观察的 3–4 mm。这里未获得新的实物测量，不能把差异归因于优化、站位、手指形变或接触力中的任何一项。

局部补偿使用七次平滑权重、七轴有界连续 IK、原姿态约束；腰部不补偿。将补偿样条与原多项式精确相加，补偿区间外 q/v/a 不变；不重新拟合整条路径，不额外消费去抖预算。

| 候选 | 最深模型 X / mm | 导数结果 | 是否可选用 |
| --- | ---: | --- | --- |
| 0 mm 对照 | 445.3377 | 通过，完全保持原数据 | 对照 |
| 3 mm，保护全部退出衔接 | 448.2926 | jerk 超限 | 否 |
| 4 mm，保护全部退出衔接 | 449.2783 | jerk 超限 | 否 |
| 3 mm，允许退出前 1.5 s 平滑过渡 | 448.2926 | 通过，jerk 约 4.9923 | 仅供审核 |
| 4 mm，同上 | 449.2775 | jerk 超限 | 否 |

“保护全部退出衔接”不是按最近点猜边界：从冻结 baseline 的退出起点定位到历史平滑区间，并保护整段跨界平滑，从现行时钟 **16.071323909 s** 起完全不修改。最深点约在 16.0123 s，补偿退回的时间很短，因此严格方案不可直接执行。

3 mm 过渡审核版的腕姿态采样变化不足 0.000001 度，IK/FK 补偿残差远小于 0.02 mm，但仍未通过实物轴线、末端余量和接触风险验证，并且修改了早期退出路径。**全部补深方案均未进入数据包，未获真机授权。**

见 `push_depth_report.json`、`depth_panels/`。不提高限速、增益或 jerk 来放行失败候选。

## 数据包与校验

- 版本：`trajectory-only-v1-db769dd65890b229`。
- release SHA256：`f4c30dc850158e98be1e024ded10506696280675fcb1c0dcc53fb33ef83954eb`。
- gear library SHA256：`13e631fcafc3f65ccba64be6a6149f775d810788febbfb9f06873d30a0b2e01c`。
- `data_only/` 仅包含三个配置 JSON 与必要冻结预览 JSON，**无 .py、无可执行文件**。
- `rollback_data/config/` 为只读核对时的原配置；`jetson_readonly/receipt.json` 留存远端配置和校验代码哈希及健康状态。
- push、pull、两条完整 gear action 的逐项哈希见 `export_report.json`；拉环 action 与 `dring_protocol.json` 保持不变。
- 已重建 lu/ul 的两条 cycle、两条完整 gear action 和库绑定；使用 Jetson 当前 GearLibrary 验证重建精确一致。
- 覆盖 count=1/2/3/8/9/17 与两种 preAction、两种 cycleMode，共 24 组；验证控制器时间到点、不提前、重复反馈不重复触发、原有块尾同值里程碑不变，最后仍保留松手与退出。
- 回归通过：新增候选/数据包/面板测试 9 项、既有停顿修整测试 9 项、快照测试 4 项，共 22 项。Qt 无硬件测试核对了面板加载、播放状态与准确时长，截图为 `push_cycle.png`、`pull_cycle.png`、`gear.png`、`depth_rejected.png`；没有进行真机或实时 RViz 运动测试。
- 尚未上传。正式激活前仍需用户确认安全失能，并重新实测无任务、controllerRunning=false、controlOwner=none、phase=DETACHED，复核在线父版本没有变化。
- `superseded_preview_metadata/` 是本轮预览标签修正前的中间包，不是待发布版本。最终包的运动数组与该中间包完全相同，仅更新了预览元数据及关联校验哈希。

## 离线预览

三个有界快照已登记默认离线 catalog；原 catalog/latest 留存在 `panel_registration_before.json`。切换“完整推进”“完整拉出”“起落架连续往返”即可读取冻结结果，不需要重新生成。逆放和补深使用独立 catalog，不替换默认候选。

离线面板显示本轮版本、基于实际新旧单元的停顿统计以及“未部署/接触待审核/导数超限已拒绝执行”状态，不再沿用旧 V5 标签。面板改动仅涉及显示，不涉及任何执行路径。

有界候选独立预览，单行命令：

`cd /home/syf/g1_upper_body_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && source scripts/tcp_a_offline_environment.sh && /usr/bin/python3 scripts/preview_trajectory_only_candidates.py`

逆放对照，单行命令：

`cd /home/syf/g1_upper_body_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && source scripts/tcp_a_offline_environment.sh && /usr/bin/python3 scripts/preview_trajectory_only_candidates.py --catalog data/rail_deployment/trajectory_only_20260929/reverse_panel`

3 mm 过渡审核版，单行命令：

`cd /home/syf/g1_upper_body_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && source scripts/tcp_a_offline_environment.sh && /usr/bin/python3 scripts/preview_trajectory_only_candidates.py --catalog data/rail_deployment/trajectory_only_20260929/depth_panels/transition_review_3mm`

这些入口只启动现有 disconnected 离线编辑面板，不启动真机控制器。独立运行时可查看曲线；机器人三维播放需要既有离线 RViz/预览节点，不能把缺少这些节点时的静态画面当成动作不运行。

回归命令，单行：

`cd /home/syf/g1_upper_body_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover -s scripts -p test_trajectory_only_candidates.py -v`
