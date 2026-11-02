# 操作台应用：第一版离线预演

## 启动与查看

```bash
/home/syf/g1_upper_body_ws/scripts/run_workbench_application.sh
```

打开 RViz 和 `G1 Workbench | OFFLINE` 控制窗。选择左右手，点击“规划进台与收臂”
或“规划插环及双向 120°”；`PLAN_READY` 后点击播放。时间轴支持离线定位查看，
暂停后可以继续播放。重置回到本次启动的参考初始姿态；换手会废弃旧计划。
规划与播放分离，不自动执行；规划失败不提供不完整路径播放。

只运行 localhost、ROS domain 77 的模拟 JointState、MoveIt 和 RViz。
不启动 arm_sdk、ARM、灵巧手 DDS、真实关节控制器或导纳示教。
启动脚本使用进程锁，防止重复启动造成多个模拟状态源。

控制窗的“初始 X 偏移 / 初始 Y 偏移”滑块可实时调整机器人站位，并支持旁边的数值框。
相对启动脚本的默认站位，范围各 ±30 cm，步进 1 mm；X 正值向台面靠近，Y 正值向机器人左侧。
柜体、拉环保持不动，RViz、FK 和 MoveIt 使用同一组离线平移关节；这些关节不加入臂规划组。
只能在初始姿态、非规划/播放时调整；任务中途须先重置。调整会清除旧计划，之后重新规划并检查碰撞。
“重置”保留本轮选定的 X/Y，重启脚本则恢复零偏移；不会修改实测关节参考或移动真实机器人。
请求接口同样支持 `{"action":"stance","x":0.02,"y":-0.04}`，单位米。

机器人现站在 7 cm 底盘上：新增 `platform_height` 参数，默认 `0.07` m，仅抬高机器人，
不抬高柜体/拉环。可用 `run_workbench_application.sh platform_height:=0.07` 显式设置。
这是离线名义高度修正，未自动标定真机 world TF；底盘长宽、脚底实际高度及新台面配准尚待实测，
没有凭空加入底盘碰撞体。真实拖拽前必须固定底盘，不能边移动底盘边沿用旧场景。

## 真机位姿记录与后续调试

独立的只读双臂记录入口（手动拖到目标后，松开拖拽键、待手臂稳定再记录）：

```bash
/home/syf/g1_upper_body_ws/scripts/record_workbench_pose.sh table_ready eth0
```

保存位置为 `logs/taught_poses/<名称>_<时间>/task_pose.json`，包括双臂/腰/下肢实测关节、
静止窗口证据，以及左右腕部相对 `torso_link` 和 `pelvis` 的名义 URDF FK 位姿。
不保存手指角度，不覆盖启动参考，不下发动作；腕坐标不是已标定的食指 TCP，文件也不是可直接执行的轨迹。
2 秒采样失败、状态过期或运动过大时拒绝记录。该记录器与控制器可以同时运行，因为没有命令输出。

2026-09-08 11:59:33 开始的只读检查成功，2159 帧、上身位置极差约 0.000216 rad、
臂电机最高温度 56°C；参考记录在 `logs/platform_readiness_20260908_115933/task_pose.json`。
这只是就绪姿态记录，**尚未抬臂或进入导纳**。

真机左臂待完成：正确的左臂链/关节映射、独立负载/重心标定及静态验证、带死手开关的低速导纳、
低负载小位移轨迹验证，然后才接台面任务。不能把右臂负载文件改名当作左臂标定完成。
右臂旧导纳 SDK 和 ROS 轨迹网关仍为独立控制器，旧导纳的超限退出逻辑没有自动继承新网关策略。
这两个常用启动脚本新增同一用户的本机互斥锁，防止同时启动；这不覆盖直接运行二进制/其他入口，
也不提供高负载姿态下的无扰控制权交接。统一命令出口和受支撑的模式切换仍需实现、验证。

## 本版动作

两手共用同一任务代码、RH56DFX 装配尺寸和任务手形参数。
启动默认左右手所有手指伸开，不读取或保存灵巧手角度，也不自动发送张手命令。
其他三指和拇指采用固定张开网格，只有食指可动；尚不代表完整灵巧手控制。
进台前在侧位显式收拢选中食指，收臂到启动位后恢复张开，另一只手保持启动手形。
完整任务为：

```text
张开侧位 -> 侧位收食指 -> 收指进台 -> 台面上方伸指 -> 斜向插入
-> +120° -> 回中 -> -120° -> 回中
-> 沿进入方向退离 -> 收食指 -> 收臂回到初始姿态 -> 恢复张开
```

食指使用社区模型的两关节和 mimic 关系，不再是永远伸直的固定网格。
食指倾角不固定为 0°，先尝试 20°、35°、50° 等候选；倾斜方向与腕部姿态一起筛选。
准备点与目标点的轴向高差为 10 cm，进入路径可以倾斜。伸指、收指的完整扫掠也做采样检查。

每只手采用自身 7 个臂关节加腰部 yaw 协同；腰部范围在离线模型内限制为 ±35°，
腰部 pitch/roll 与下肢锁定。另一只手不主动运动，但跟随躯干移动并参与环境碰撞检查。
没有修改真机 URDF、保护阈值、控制器增益或重力补偿标定。

## 两点目标约定

- `axis_center`：旋转轴与轴顶平面的交点，不是 D 环内部中心。
- `axis_quaternion_xyzw`：局部 +Z 为旋转轴正向。
- `entry_center`：D 环内部的食指插入目标点，与轴心位于轴顶平面。
- 半径为两点差在该平面内的长度。正角按右手定则绕 +Z；不要依靠屏幕左右理解正负角。
- 默认 CAD 模拟半径 40 mm，目标尚未经过视觉或实物标定。

当前默认目标在 `assets/workbench/task/target.json`。
站位和柜体保持上一版几何关系；柜体底与脚底同平面，当前胯部前表面到桌沿约 11 cm。
当前参考初态为 2026-09-08 10:52:01（Asia/Shanghai）的真机 `/lowstate` 静止窗口采样，
用户确认已进入运控准备位，但没有自动验证 R2+A 按键/FSM。
文件为 `assets/workbench/task_start_pose.json`，原历史调整位姿已备份。
腰 roll/pitch 的两个槽位无有效电机数据，明确作为锁定为零的模型假设，不能称为实测。
机器人 world 位姿、脚底高度和柜体关系仍沿用离线场景，未由本次关节采样重新标定。
启动参数 `initial_pose_file:=...` 同时供模型构建和应用初态使用，避免两者引用不同文件。

视觉预留 `/workbench/vision_target` (`geometry_msgs/PoseArray`)：
`poses[0]` 为轴顶中心及方向，`poses[1].position` 为环内中心，单位米。
要求 `header.frame_id=world`，ROS 时间戳有效期 2 秒，两个点轴向高度差不超过 3 mm。
**本版固定 CAD 场景尚不支持重新配准：轴位姿与当前模型不一致会拒绝，不会只移动标记。**
相机 TF 转换、目标置信度/协方差、任意目标的场景配准留在后续视觉适配层实现。

## 任务接口

`/workbench/request` 使用 `std_msgs/String` 承载 JSON，状态在 `/workbench/status`。

```json
{"action":"plan","arm":"right","scope":"full"}
{"action":"play"}
{"action":"pause"}
{"action":"seek","fraction":0.5}
{"action":"cancel"}
{"action":"reset"}
```

`scope=entry` 只规划进台和收臂。`target` 请求使用与视觉相同的几何检查。
执行中不可换手/改目标；离线暂停在中途时，必须重置或完成旧任务后才能重新规划。
最近成功的计划保存在 `assets/workbench/task/last_{left,right}_{entry,full}_plan.json`。
旧文件只可作为逆解初值，每次仍需重新求解并检查，不能直接当作当前可执行计划。

## 验证与边界

MoveIt 检查模型碰撞、关节限位；随后对相邻轨迹点间按关节步长不超过 0.01 rad 再采样检查，
包含转动拉环和活动食指。圆弧阶段检查实际指尖 FK 与目标圆弧的误差，超过 2 mm 拒绝。
接触阶段只放行选中食指远端与拉环，不放行手掌或整张桌子。CAD 裁分、社区手模型、
装配偏差与指内橡胶/掌体的建模重叠仍需要实物校核；采样检查不是连续碰撞安全证明。

**当前播放是几何插值动画，不是动力学仿真或真机运动轨迹。**
界面的倍速只改变离线播放速度，不能证明真实力矩、负载、接触力或加速度/jerk 安全。
OMPL 进出路径使用现有时间处理适配器，但本版统一动画时间不是完整任务的硬件时间参数化。
插入和带动拉环也是运动学预演，未模拟卡滞、滑动或接触确认。

同事控制仓库已下载到 `third_party/g1-rh56dfx-control`。下一阶段才接入 HandClient、
臂命令唯一出口、真实手指标定、限速/限加速度/限 jerk 的整段轨迹、接触确认与退出策略。
`run_right_arm_remote_teach.sh eth0` 保持原样，尚未并入本应用；绝不能与真机轨迹网关同时启动。
左右手的运动学接口相同不代表右臂实测重力补偿标定可以直接用于左臂。

2026-09-08 已做只读可达性检查：左右 RH56DFX GetState 均超时（3104），头部 videohub
返回有效 1920×1080 JPEG；当前 domain 0 / eth0 未发现深度、CameraInfo 和相机 TF。
详见 `logs/hardware_readiness_20260908_001/REPORT.md`，包含深度像素到 `torso_link` 的转换约定。
只读检查工具为 `scripts/capture_workbench_start_pose.py` 和 `scripts/run_hand_camera_readonly.sh`；
后者仅保存服务可达性与相机图像，不保存手指角度，不启动 hand_service 或发送动作。

离线回归：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 scripts/check_workbench_application.py --arm left --scope full --play
/usr/bin/python3 scripts/check_workbench_application.py --arm right --scope full --play
/usr/bin/python3 -m unittest discover -s scripts -p 'test_workbench_task.py' -v
/usr/bin/python3 -m unittest discover -s scripts -p 'test_preview_payload.py' -v
```

## 随机站位离线测试

`scripts/workbench_robustness.py` 使用实际应用规划器，固定关节初态、柜体和拉环，
只改变机器人整体初始 x/y。默认种子为 `20260908`，100 个站位，每个站位左右手各规划一次完整任务。
x 相对原位置为 -6 至 +10 cm（正方向靠近台面），y 为左右各 12 cm；85 个样本靠近台面，
其中 60 个前移 4 至 10 cm。名义胯部前平面间隙不能当作全身到柜体的最小距离。

测试单独使用 localhost ROS domain 78 至 85，不占用应用 domain 77，不启用真实控制或手部 DDS。
需要八个独立 MoveIt 服务；`run_workbench_robustness.sh server <run_directory> <shard>`
启动对应服务，`worker <run_directory> <shard>` 执行分片，`shard` 为 0 至 7。
准备输入时由 `prepare --root <new_directory> --snapshot <move_group_parameter_dump>`
保存场景、初态、原有成功逆解初值和采样清单；已存在的实验目录不会被覆盖。
重新运行 worker 会跳过已完成记录，不应删除旧结果来隐藏失败。

每次规划限时 180 秒。初始碰撞拒绝、搜索未找到完整路径、超时、路径/状态 Bug 和通信异常分别记录。
成功计划还会进行额外关节插值碰撞采样、指尖 FK 交叉核对、圆弧误差、完整阶段、固定站位和回原位检查。
时间预算仅用于离线测试，不改变应用规划策略或真实机器人的保护策略。

当前批次记录在 `logs/workbench_robustness_20260908_003/`。可在运行中或结束后生成报告：

```bash
.venv-cad/bin/python scripts/report_workbench_robustness.py \
  logs/workbench_robustness_20260908_003 --plot
```

`REPORT.md` 标明是否全部完成，`stances.csv` 为逐站位结果，`stance_results.png` 为分布图，
`results/` 保留逐臂 JSON 与成功计划的压缩副本，`provenance.json` 记录输入和代码哈希。
超时不代表不可达，单点通过不代表整个邻域安全；本轮不测试站位朝向、视觉噪声或真实接触动力学。

该批次已完成：左手 87/100、右手 50/100 完整通过；13 个站位有初始髋部/柜体碰撞，
右手另外 33 次未找到完整路径、4 次超时。本轮未检出路径/状态 Bug，完整统计以报告为准。
URDF 的外部 STL 网格仍引用当前工作区，复测时也需保持这些资产一致。

**该批次是旧历史初态、收拢手形的基线，不覆盖 2026-09-08 新采集初态和默认张开手形。**
当前已通过 34 个单元测试及无进程场景构建检查，尚未重跑新配置的完整规划/100 站位测试，
也没有验证新配置的真机碰撞、接触和动力学安全。

## 左臂真机试调入口（2026-09-08）

已按用户授权复制右臂导纳/末端参数到左臂。负载为未验证的复制初值，
重力使用左臂 KDL 链，不直接复制右臂力矩。先运行：

```bash
/home/syf/g1_upper_body_ws/scripts/run_left_arm_remote_teach.sh eth0
```

F1+X 长按 0.8 秒接管并等待准备完成，按住物理 F3 拖动左臂，松开刹停保持，
F1+A 记录。固定底盘，准备位附近小幅测试；旧 SDK 故障退出逻辑仍保留。
本轮未发送真机动作，未完成实际导纳测试。

末端入口 `scripts/run_left_arm_endpoint_motion_server.sh eth0` 已准备但尚未联调完成；
隔离网关测试的模拟 LowState 接收仍待排查。先完成导纳评估，不同时运行两个控制器。
50 项单元测试和编译通过不代表真机验证通过。详情见
`logs/left_arm_adaptation_20260908_001/REPORT.md`。

## 统一手部几何（2026-09-08 后续更新）

`assets/workbench/payload_preview.json` 现在同时用于真实末端 launch 和工作台：
左右 RH56DFX 静态张开模型，连接件长度 4 cm、直径 6 cm；碰撞使用手模型网格
及连接圆柱，而不是旧真机入口的 30 cm 长盒。真实规划器、RViz 与 TF 描述同步。
动态重力仍使用原负载拟合，不能将本次几何修改视为重新标定。

此修改需要重启规划/显示会话，不热改正在支撑机器人的控制器。旧 tool0 目标及
旧轨迹需要重新计算；保存的腕部参考可通过新的掌根变换重新解析。
当前静态张开显示不依赖手 DDS 在线，也不代表真实手指角度。本轮只读检查未发现
同事 hand_service 的 RPC 接收端，左右状态查询超时，未启动任何手部硬件服务。

## 食指任务的小幅姿态自由度（2026-09-09）

离线完整任务仍先搜索可用的整体插入姿态。选定后，插入、圆弧、退出阶段允许
食指 TCP 相对该参考方向偏转最多 5 度、绕自身轴转动最多 10 度，相邻路径点的
三维旋转变化不超过 2 度。左右手使用同一策略；不是单独转动拇指电机。

目标 JSON 可增加 `orientation_tolerance`：

```json
{"tilt_deg": 5.0, "roll_deg": 10.0, "step_deg": 2.0}
```

先尝试保留上一点的姿态；关节变化较大、接近限位或逆解失败时，搜索小幅局部
姿态变化，按关节变化、腕部变化和关节限位余量评分。偏转上限以本次选定参考
为准，不允许沿圆弧累计漂移。整体参考姿态的离散搜索仍可能选择不同拇指朝向。

保持原碰撞策略和完整路径验证，增加插入/退出插值位置误差检查（2 mm）以及
接触阶段插值姿态范围检查；圆弧误差仍为 2 mm，失败不给出可播放的部分计划。
该策略不是连续全局姿态优化，也不是速度/加速度/加加速度规划，更未完成真机
接触验证。实测视觉点仍只是预览，尚未注册进离线 CAD 规划场景。

`scripts/check_workbench_orientation.py` 可在独立 localhost domain 83 中构建
临时场景并测试双臂完整规划，禁止轨迹执行、不覆盖默认模型或历史计划。
`--stance-x/--stance-y` 仅改变测试场景。原默认站位测试发现保存的张开食指与
CAD 台面初始碰撞，已正确拒绝；不能宣称当前启动姿态的完整任务已经可执行。
