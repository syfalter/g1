# 新会话 Prompt：仅微调轨迹，冻结业务与控制逻辑

> 2026-09-30 最新选中版本：用户确认失能并指定9mm，实时核验DETACHED及7mm配置后，已激活 `session-push-9mm-0407b06c6a1e` 并重启 `eln_executor.service`。release SHA256为 `6e18941fafceb2efaf248ce158125fd26d01668b42fb41b071d66d1d0496c84d`，API/native的release、gear、dring哈希均一致；重启后仍DETACHED，未初始化或发送运动指令。9mm相对原基线，比前一7mm多2mm；其他轨迹与业务逻辑不变。旧7mm配置备份于运行目录 `data/deployment_backups/session-trial-1790738385883409753`。Jetson激活目录为 `trajectory_candidates/push9-activation-20260930`，本地凭据为 `data/rail_deployment/push_depth_trial_series_20260929/trial_9mm/activation_receipt_20260930.json`。候选源版本保持原始3mm父版本溯源，安装前另核对实际7mm激活基线及全部其他配置/已审核运行时哈希。物理接触安全仍待人工测试。本条取代下文7mm/3mm当前选中状态。

> 2026-09-29 最新选中版本：用户再次确认失能并指定7mm后，安装器实时验证DETACHED及父配置哈希，已激活 `session-push-7mm-ed4fd9bb98e1` 并重启执行服务。release SHA256 `9fbd5057316b63e72de41a098714e4259d0bebd6ce237f326b144bb81da3e16b`，API/native的release及两库哈希一致；仍未使能、未发送运动指令。仅push改为相对原基线补深7mm（比前一3mm版增加4mm）；其他动作、快速逆放起落架及业务逻辑不变。旧3mm选中配置备份于运行目录 `data/deployment_backups/session-trial-1790680024978667157`，凭据为 `data/rail_deployment/push_depth_trial_series_20260929/trial_7mm/activation_receipt.json`。4/5/6/8/9mm仍是未激活候选；物理验证仍未完成。本条取代下面“仍选中3mm”的历史状态。

> 2026-09-29 补深阶梯试验：用户反馈3mm实物推进仍不足，要求切4mm并准备5/6/7/8/9mm。已核对API、native和选中配置确实是上述3mm发布版。4–9mm均已生成、通过原连续导数限值、上传至 Jetson `trajectory_candidates/push-depth-trial-series-20260929/trial_{N}mm`，每包4个数据JSON哈希已核验。仅push变化，拉出、起落架快速逆放、拉环、手型与业务逻辑不变。当前仍 `initialized=true / ARMED_HOLD`，**未切4mm、未重启、未发送运动**，需用户安全失能后重新读取状态才能激活。幅度相对0mm原基线，非逐次叠加；5–9mm用峰值后2秒退出过渡，物理轴线、接触和机械余量仍未验证。见 `data/rail_deployment/push_depth_trial_series_20260929/report.json`、`upload_receipt.json`，离线回归2项通过。每包目前以3mm发布为父版本，后续逐级切换必须重新核对/更新父版本证据，不能绕过安装器检查。当前实现启动时固定加载release，上传候选不需要失能，选中版本切换仍需安全失能后重启，不支持直接替换文件热加载。

> 2026-09-29 最新激活记录：用户确认安全失能并授权测试后，已将 `session-trial-fast-reverse-2f8521da6a75855a` 设为 Jetson 当前选中版本并重启 `eln_executor.service`。包含推进补深3mm、pull接近去迟滞+沿X少拉5mm+优化退出、起落架8.963196924秒上行及其精确快速逆放下行。API与native的release/gear/dring哈希均核验一致，重启后保持DETACHED，未初始化、未发送运动指令。旧配置备份于运行目录 `data/deployment_backups/session-trial-1790678782287476392`。权威凭据：`data/rail_deployment/session_trial_activation_20260929/activation_receipt.json`；release SHA256为 `d3f62effa025e655f8ac0e4ba529cf38f2415e8e3fd577833bb4745694380e5e`。仅数据变更，物理接触安全仍待人工测试，`physical_verified=false`。本记录取代下文“仍未激活”的历史状态。

> 2026-09-29 17:31 CST 完整清单更新：此前漏传的起落架有界上/下行、周期、两条完整动作及配套库已补到 Jetson `trajectory_candidates/session-bounded-20260929T093102Z`，与选中的推进3mm、最新pull同目录，18个JSON哈希核验通过，旧版保留，**仍未激活**。已有9.10s逆放及反向本次8.96s上行，与上下接近/退出边界均通过C2/导数/进度模拟，但逆放未被自动选用，接触风险未消除。完整清单及“逆放后必须接下杆退出”等结论见 `docs/session_trajectory_inventory_20260929.md`；优先读此最新记录，不要沿用下文旧阶段“起落架未上传”的状态。

> 2026-09-29 17:09 CST 最新进展：用户选推进补深 3 mm，并要求拉出沿 X 少约 5 mm（允许截尾），随后要求减少抓握前迟滞。已生成 pull 截尾+接近去迟滞组合：接触 4.315519→3.882505 s，接近 22.007150→17.000000 s，完整预览31.640874 s；原手型等待/抓握位姿保留。截尾导致松手 X+5 mm，Y-2.846930/Z-0.995691 mm、姿态约0.848312度变化。推进3mm与新pull的待审核数据已上传 Jetson `trajectory_candidates/push3-pullshort5-ready-20260929T090936Z`，旧版保留；**未激活，复查时实机 RUNNING，禁止按历史失能状态自动重启**。详见结果文档顶部与 `data/rail_deployment/pull_ready_shorter5_20260929/report.json`。

> 2026-09-29 后续范围更新：用户已允许推进补深后的早期退出路径有少许变化。标准 `start_task_panel.sh --offline` 的“优化对比”已新增无箭头毫米级末端轨迹，以及“推进补深 0 / 3 / 4 mm”比较组。3 mm 使用峰值后 1.50 s 过渡、4 mm 使用 1.75 s 过渡，均通过原导数限值；实物轴线/机械余量/接触安全未确认，未入发布包、未部署。下文“保持退出段不变”是原约束，已按上述授权局部放宽；原有界低速优化仍单独保留。详见 `docs/trajectory_only_results_20260929.md` 顶部更新及 `data/rail_deployment/push_depth_exit_v2_20260929/report.json`。

请接手 `/home/syf/g1_upper_body_ws` 的轨迹优化工作。先仔细阅读下面列出的代码和报告，不要重新开发业务执行链，也不要把整个 WSL 代码目录覆盖到 Jetson。

## 用户目标与范围

业务逻辑、上游请求解析、连续组合、完成次数回调已由用户确认可用。后续只需要：在 WSL 离线调整轨迹、预览与对比、导出版本，然后将优化后的轨迹数据上传至 Jetson。

优先处理两项：
1. 起落架上下往返仍偏慢。评估能否用较短的“下到上”轨迹反向播放，替代较长的“上到下”；同时比较原路径约束内的局部修整方案。
2. 滑轨推进时，实物最深推动位置仍差约 3–4 mm。分析能否对推进接触段做局部补深，而不是全轨迹平移，且不要破坏退出和准备位。

不要顺带修改协议、任务映射、count 语义、回调字段/发送/重试、控制权交接、自动恢复、手指顺序、电机增益或网关限值。恢复初始站姿/安全下电接口虽有历史本地代码但未接入当前服务，不属于本会话范围。

## 当前已部署基线（2026-09-29）

- Jetson：`unitree@192.168.123.164`。
- 运行根目录：`/home/unitree/rh56dfx/workspace/execution/action_runtime_v3`。
- 业务协议与通信服务：`/home/unitree/rh56dfx/workspace/communication`。
- `eln_executor.service` 是真实执行服务；`eln_interface.service` 对外 8090，转发到内部 8795；`eln_monitor.service` 为 8092 观察界面。
- 选中版本：`gear-bounded-v6-330f290121485a5e`。
- release SHA256：`f44ea3780a075423b7b760518378435226d8288fadfd22f81080a73f027affd7`。
- gear library SHA256：`014f62392db8fdaa85624469ffd24f7efac5f814fa4bb161b75f287718ec57b1`。
- D-ring library SHA256：`c663da6476a08b31484b4e3a56856927306104eb58134b6f3f71b60eef721ecf`。
- 上述是交接时基线，不是未来实时状态。开工先只读核对健康接口、配置和已安装文件；其他窗口可能继续工作。

真实配置在 Jetson：
`config/selected_bundle.json`、`config/gear_protocol.json`、`config/dring_protocol.json`。

WSL 的对应轨迹数据在：
`data/rail_deployment/gear_reverse_latency_20260928/` 下同名文件，其中选中包叫 `bundle.json`。

部署凭据必须优先看：
- `data/rail_deployment/gear_reverse_latency_20260928/activation_receipt.json`
- `data/rail_deployment/cycle_progress_20260929/activation_receipt.json`

注意：前一个目录的 `build_report.json` 中 `deployed:false` 是构建时状态，后来已经部署；以 activation receipt 为准。

## 必须保留的已完成行为

- 上电、R2+A 本身不自动接管双臂；仍需明确初始化请求。
- 固定发布版在服务启动时完整校验并固定；请求时复用，不重复整包校验。
- 只有首个 `align to pinned task start` 且新鲜静止反馈证明所有目标关节误差不超过 0.005 rad 时才跳过重复对齐。
- 拉环单次左拨+右拨算一次；多次用 `entry + cycle*(count-1) + final`，中间用截短片段衔接，最后才使用原右拨退出尾段。
- 拉环 1..8 次在一个 recorded block 中执行，超过 8 次分有界块，但整个任务仅一次接近、一次最终退出。
- 逐次 progress 已修复：根据真实控制器轨迹时间上报，不依赖墙钟计时，不为了回调切断轨迹。
- 拉环的中间 cycle 包含返回下一次入口的衔接；最后一个 final 包含退出，其最后一次计数仍保留动作结果/到位确认。
- 起落架按已经定义好的周期边界逐次加一；完成周期后仍需松手退出，才最终成功。
- 控制器反馈时钟会随跟踪减速/暂停，不能改成定时器“估算完成”。
- 失败任务保持 FAILED，不重放；符合已确认检查点和安全条件时自动恢复准备位，新请求可等待恢复结束后执行。
- 起落架抓握：四指先 0.5 秒，拇指后 0.5 秒；松开顺序相反。已取消指位必须到目标的要求，但保留命令确认、新鲜健康反馈、故障和停止检查。
- 外部 WOW/机械锁信号不参与成功判定，当前是运动序列完成语义。

### 起落架 count 不能重新解释

`GEAR_OPERATION` 的 count 指“收起→放下”周期数：
- `preAction=NONE`：从 LOWER 开始。
- `preAction=EXTEND_DOWNLOCK`：从 UPPER 开始，先 UPPER→LOWER，这段不计次数。
- `cycleMode=FULL_CYCLE`：每次 LOWER→UPPER→LOWER。
- `cycleMode=FINAL_RETRACT_ONLY`：最后一次省去放下，结束于 UPPER。

例如 count=1：NONE/FULL 是 L→U→L；NONE/FINAL 是 L→U；EXTEND/FULL 是 U→L→U→L；EXTEND/FINAL 是 U→L→U。

## 必读代码：离线轨迹与速度策略

以下路径均相对 WSL repo：
- `scripts/task_offline_optimizer.py`：Policy、按阶段路径误差预算、连续多项式导数校验。
- `scripts/task_trajectory_speed.py`、`scripts/task_trajectory.py`：实际关节速度/加速度限制来源、插值与峰值、腰部包络校验。
- `scripts/task_composition_smoothing.py`、`scripts/task_smooth_timing.py`：局部空间平滑、速度重整，不要只缩放时间戳。
- `scripts/task_stop_blending.py`：删除/拼接造成的内部零速点处理、低速静止区间压缩；默认七次 C3，另有五次 C2 候选，均需路径与导数验证。
- `scripts/task_offline_optimizer_cases.py`：原片段、组合单元、flow 生成过程。
- `scripts/task_composition_preview.py`、`scripts/task_rail_composition.py`：组合、截短版与原版尾段、接近/退出处理。
- `scripts/task_optimizer_snapshot.py`：不可混淆的预览快照、catalog/latest、模型校验。
- `scripts/task_composition_panel.py`：离线组合预览面板。
- `scripts/build_gear_lowering_release.py`：已有的有界下行发布脚本，仅参考轨迹重建与配对验证，不要重新覆盖运行时代码。
- `scripts/export_dring_protocol.py`：拉环循环单元导出与完整单次一致性验证。

当前 Policy 默认 jerk=5 rad/s^3，接触 TCP 偏差 1 mm、非接触 5 mm，空间窗口 0.02。
对应接触 joint 预算 0.003 rad、姿态预算 0.5 度；非接触按 5 倍预算。
这些是当前软件优化策略，不是已获厂商证明的电机 jerk 极限；不得把它们描述为硬件性能上限。
速度/加速度必须从当前实际 planning_config 和 motion_limits 读取，不要只依赖默认值。
腰部当前轨迹上限 0.35 rad/s，网关 100 Hz，单步界限 0.006 rad；本任务不提高这些限制。
验证应覆盖连续插值峰值，而不只是离散采样点，且不能在多个平滑阶段重复花费同一份几何容差。

## 必读材料：起落架逆轨迹已做过的分析

- `scripts/review_gear_reverse.py`
- `scripts/prepare_gear_lowering_candidate.py`
- `data/rail_deployment/gear_reverse_latency_20260928/reverse_review.json`
- 同目录 `bounded_lowering.json`、`bounded_lowering_c2.json`、`build_report.json`
- 同目录 `previews/bounded/` 和 `previews/reverse/`：独立候选快照，不能假设已注册进当前面板 catalog。

已知结果：
- 下到上 lu：9.099692908 秒。
- 旧上到下 ul：16.846077752 秒；当前线上有界优化版为 16.354511176 秒，仅省约 0.49 秒。
- 把 lu 反向，时长仍为 9.099692908 秒；端点与旧 ul 差约 1.39e-16 rad。
- 正确逆放需反转采样顺序和时间、速度取反、加速度不变；已有 `automatic_recovery.reversed_path` 可参考数学实现，不能因此改变业务恢复策略。
- 逆放候选导数验证已通过：峰值速度约 0.3041 rad/s、加速度 1.0459 rad/s²、jerk 4.9257 rad/s³。
- 但是两条中间接触路径的采样双向最近距离约 18.98 mm，近邻姿态差约 18.78 度。该指标是诊断，不是严格对应或碰撞证明。
- 因此不能只凭端点一样就称它等价、安全或直接上线。当前逆放候选没有选中、没有真机验证。
- 旧 ul 腕 roll 累积转动约 0.1568 rad，逆放约 0.0066 rad；低于 5 mm/s 的 TCP 时间旧版约 7.10 秒，逆放约 2.47 秒。应重点检查不必要腕部运动与低速拖尾。
- 当前有界版最大报告 TCP 偏差约 0.933 mm，仍在原接触 1 mm 预算内。

请优先复用上述候选与分析，离线检查接触几何、姿态、握持约束、是否存在方向相关接触限制，再决定逆放能否成为新候选。可以制作并预览候选，但真机验证/替换须另行确认。

## 必读材料：滑轨推进补深 3–4 mm

- `scripts/review_rail_accuracy.py`
- `data/rail_deployment/rail_accuracy_20260928/analysis.json`
- 同目录 `logs/`、`native_jobs.json`
- 当前 push 的冻结快照从 `bundle.json` 的 `payload.actions.push.source` 读取，不要拿面板当前选中项自动替代。

已有离线结论：原版最深 x 约 445.346466 mm，flow 约 445.337709 mm，相差仅约 0.00876 mm；不能据此说优化少推了 3–4 mm。
历史日志在峰值附近的关节 FK 前向误差约 -0.1067 mm，也不足以解释实物误差。
这些量在 URDF root/pelvis 的 +X 前向系下，且指尖采用命令手型近似，不包含实际站位变化、物体位置、手指弹性/接触形变，不能等同于实物测量。

用户希望能够通过编辑轨迹补足实物进深。请：
1. 明确实际滑轨轴线、机器人前方与模型坐标的对应，不假设任意 +X 都是正确补偿方向。
2. 比较当前命令轨迹、执行日志、实物观测，区分几何不足与跟踪/接触误差，不能无依据归因。
3. 制作局部推进末段补深候选，补偿幅度可选 0/3/4 mm，对前后段使用平滑权重与一致的姿态/IK 连续性；不要突然改一个点或整条路径平移。
4. 补深是有意修改任务几何，不是原路径 1 mm 平滑预算允许的“去抖”。明确列出补偿与平滑各自带来的变化。
5. 保持准备位、起始衔接、退出段、手型时序不变；验证腕姿态、关节限位、导数、腰部包络及接触风险。
6. 不强推未知硬限位，不用提高电机限速或增益掩盖不到位。先预览候选，实物试验需要用户确认。

## 发布原则：以后只上传轨迹数据

业务代码已稳定，本会话只发布新的录制轨迹与必要的配套元数据，不部署整份旧 runtime 包：
- 更新 `selected_bundle.json` 内目标 action 的 plan/program/导数数组/时间/校验哈希与版本。
- 如果变更起落架 ul/lu，要重建 cycle_lower/cycle_upper、两条完整组合及 `gear_protocol.json` 的 units、bindings、snapshot/model 校验信息，验证完整轨迹与单元重建一致。
- 未修改的拉环保留 `dring_protocol.json`；若将来修改拉环，再同步其单次动作与 entry/cycle/final 绑定，不能只替换单个 JSON 中的一段数组。
- 更新轨迹时不修改 count 和控制器时间驱动的 progress 算法；改变单元时长后应离线验证进度检查点随新时长正确生成。
- 保留其他所有 action 的内容，提供逐项差异与 hashes，避免误换别的候选。
- 发布是“数据包＋必要校验/绑定信息”，不是完全无校验的文件拷贝；正式数据仍需有效性检查。
- Python 轨迹数据变更通常不需要重新编译 C++。服务当前启动时固定数据版本，换包后需要在安全失能状态重载/重启执行服务才能生效。
- 当前 service/runtime 以 Jetson 最新只读快照为准。WSL `deployment/rail_jetson`、历史 staging 和 Jetson 之间存在版本差异，尤其不能重新运行历史整包安装脚本覆盖线上进度桥接。

真实执行链需要了解但不要随意修改的文件：Jetson 的 `dring_protocol.py`、`gear_protocol.py`、`gear_progress.py`、`executor.py`、`ros_transport.py`、`native_worker.py`、`native_runtime.py`、`cloud_executor.py`、`production_api.py`、`release_validation_cache.py`。
最新计数补丁在 WSL `data/rail_deployment/cycle_progress_20260929/runtime/`。它比前一个合并包的 runtime 更新；曾发生旧执行器覆盖逐次进度桥接的回归，务必避免重现。

## 工作方式与交付

- 首先离线工作，读冻结版本与已有报告；不要发送机器人初始化、运动或测试任务来“检查”。
- 用户之前确认失能仅适用于当时的更新，不是本新会话的持续授权。部署前重新确认安全失能，并实测无任务、controllerRunning=false、controlOwner=none、phase=DETACHED。
- 修改 WSL 文件不要覆盖其他窗口更改；该目录可能不是 git 仓库，先检查再操作。
- 不默认启动 WSL 真机控制器，避免与 Jetson 竞争。面板仅预览可使用离线模式。
- 在离线面板提供明确版本的新候选，报告时长必须与用户点击任务实际看到的快照一致，不强迫用户重新生成才能看到结果。
- 提供对比：每段耗时、TCP 路径/姿态差、低速拖尾、腕部累计移动、速度/加速度/jerk 峰值、末端最深推进位置。
- 优化候选通过验证并获用户认可后，仅上传轨迹数据包，备份原数据，安全重启，核对 API/native 加载的版本与库哈希。保留回滚数据，不自动初始化。
- 面向用户的 bash 命令全部写成单行。

请先从“当前两条起落架行程与已有逆放候选的差异”和“push 局部补深候选的定义”着手。不要重新讨论已定业务接口，也不要把未通过的候选说成已上线或已实测。
