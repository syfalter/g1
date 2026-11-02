# 实验 A：同一点、多腕姿诊断

## 启动

当前软件环境为ROS2 Humble、Qt5/RViz2和colcon。先通过原工程正常启动任务控制界面：

```bash
bash /home/syf/g1_upper_body_ws/scripts/start_task_panel.sh
```

按原流程核对现场安全、控制权及实测状态，人工确认接管/ARM。启动不自动ARM。再启动实验面板：

```bash
bash /home/syf/g1_upper_body_ws/scripts/start_tcp_a.sh
```

此命令打开诊断RViz，但不另启机器人控制器。也可以在已有RViz的Panels/Add New Panel中添加`g1_tcp_diagnostics/TcpAPanel`，后端用`start_tcp_a.sh --backend-only`；Marker topic为`/g1/tcp_a/markers`，Fixed Frame设`pelvis`。实际插件显示名称以RViz列表为准。

本轮结束后助手已释放控制并关闭界面，不会自动重开。新腰导纳尚未真机调试，原轨迹连续性离线脚本另有两项失败，见结果分析文档；不能把编译成功当整机验收。

## 五个页面

1. 布置引导：原控制器连接、伸食指、导纳选项和恢复当前姿态保持。新增“包含腰yaw”选项，默认不选；勾选后仅能从静止保持进入，需确认躯干/双臂空间和支撑。原任务面板也有“双臂＋腰yaw导纳”按钮。
2. 接触采样：选左/右手并新建会话；填写实体触点ID；只选“拟合组/独立验证组”，无第一/第二批。核对真实LowState、固定pelvis、固定手型。每条单独勾真实轻触和无压缩/滑动，点击“采样1秒取平均”。
3. 样本残差：点选高亮、查看原始状态和腰角、排除/恢复、更改用途。排除不删除数据。已有会话分组和旧phase元数据保留，不自动改组。看过结果再改变分组后须重新采独立验证。
4. 求解导出：求解固定手型三维TCP；预览候选，查看拟合及验证分开指标；保存/导出生成新文件。候选不自动应用。
5. 跟踪系统：查看数据来源、新鲜度、稳定门限和命令参考可用性；命令源非唯一时显示缺失，不填0，也不扣除跟踪差后美化拟合。

## 实体布置与操作

pelvis是腰yaw下方的刚体坐标系，不是腰关节。所有FK包括motor12实测腰yaw和所选臂实测关节，源为/lowstate；不用演示/target joint_states。当前模型腰roll/pitch固定，工具不将其标为实测，也不开放导纳。

固定机器人支撑及参考块，在食指最末端正前方标记一个小点，每次用它轻触参考块同一个定位点。板可以在相机范围内，但本实验没有RGB/深度、板检测或相机外参输入；未确认的ChArUco型号不影响A，也不被写成已测几何。

无接触且ARMED_HOLD时，先设置所选手伸食指，等待手部反馈完成并目视核对。选是否包含腰，再确认导纳。必须等待TASK_ADMITTANCE；转腰还必须看到“腰yaw导纳已启用”。没启用不可强扭。另一手也进入导纳，需要空间及支撑。手型由原HandControl/hand_service执行；采样后端只读。采样前可以启用附加手DDS读取，但执行器反馈不等于独立指骨实测角。

每次离开参考点后改变腰/腕姿，再轻触并保持，点击一秒采样。收集建议8至12条多轴姿态拟合和4至6条额外姿态验证，两组都可以转腰；腰角不是分组依据。先同姿态重复3次可看接触重复性。数量够不等于激励充分，SVD秩/条件数不合格会拒绝求解。

采样平均实测关节后做FK，并另存逐帧FK点平均/标准差及完整窗口；记录方法区别。不把不同接触姿态平均成一个样本。反馈不新鲜、缺失、位置漂移或控制阶段改变会拒绝采样，拒绝原因可见。稳定判断使用实际关节位置窗口，不用有噪声的单帧dq峰值假装一定在运动。

结束时先离开触点、保持静止，再“恢复当前姿态保持”。这不是回初始目标。需要退出整个系统时先人工回安全姿态，通过原面板关闭应用或运行`stop_task_panel.sh`正常释放，再关闭诊断RViz。不要强杀仍持有控制权的进程。

## 颜色与计算

蓝=原模型点，绿=候选TCP点，紫=估计共同触点，红=绿点到估计共同触点残差；球=拟合，方=验证，灰=排除，黄=选择。蓝点不是相机点，紫点不是实体测量真值，红线不是力。机器人和网格不是独立精度基准。

T_A_B把B系坐标变到A系；m/rad，矩阵行优先，四元数xyzw。B=pelvis，W=所选手wrist_yaw_link（腕后加长件之前）。拟合`R_i*s+t_i=c`，只解固定手型位置s及同点c，不解六维TCP，也不同时优化相机外参。验证同一实体点时固定拟合得到的中心；新点重新居中只能检验一致性。

## 保存与离线复算

`data/tcp_a/<session_id>`中保留实际状态、来源、时间、frame、模型hash、手型假设、窗口统计、原始窗口、排除与用途、拟合结果。导出子目录包括JSON、CSV、candidate_tcp.yaml和REPORT.md，均新建，不覆盖在线配置。

可只查看上一轮而不连接真机：

```bash
bash /home/syf/g1_upper_body_ws/scripts/start_tcp_a.sh --offline \
  --session /home/syf/g1_upper_body_ws/data/tcp_a/8b10e0ad98f74492a59cb07569b594c7/export_20260912_181011_805249/session_20260912_181011_805565.json
```

离线transport固定domain96/localhost，不订阅硬件；归档mode=live仍表示原始数据来源，不能据此开启控制。真机继续旧会话请用面板“加载”，等待模型一致性核验；当前`--session`启动加载只用于离线。改变手型/安装应新建会话。

## 构建和针对性检查

```bash
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source /home/syf/g1_upper_body_ws/install/setup.bash
cd /home/syf/g1_upper_body_ws
colcon build --packages-select g1_unitree_control g1_tcp_diagnostics --executor sequential
cd scripts
/usr/bin/python3 -m unittest test_tcp_a test_tcp_a_node test_task_tcp_a_controls test_task_admittance test_task_console
```

新增腰导纳服务沿用ResumeTaskHold的stop_epoch请求类型，路径`/g1/arm_gateway/stop_task_with_waist`，只有原任务界面明确确认后调用；普通stop_task行为不变。参数为保守软件初值，不冒充厂商标准或已验证人机交互参数。保留原过载/温度/断流/限位/释放机制，不自动重置故障。模型和官方外参未变，没有候选TF发布。后续要手工应用TCP需独立验证通过并另行备份原手模型；这轮候选不建议应用。

上一轮详细解释及测试限制：`logs/tcp_a_review_20260912/REPORT.md`。
