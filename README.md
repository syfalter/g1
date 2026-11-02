# G1 上肢控制

基于 ROS 2 Humble、MoveIt 2 和 Unitree SDK2 的 G1 上肢操作工作区，包含双臂运动学、轨迹规划、重力与负载补偿、导纳示教、视觉标定、轨迹编辑和 Jetson 任务执行。主要应用为 D 环拨转、滑轨推拉和起落架杆件操作。

本仓库整理自 `/home/syf/g1_upper_body_ws`，包含 2026 年 8 至 9 月的开发代码与实验记录，以及 10 月的成果报告。

## 系统结构

```text
关节状态 / RGB-D / 示教录制
          |
    模型、标定与任务几何
          |
    IK / MoveIt / 轨迹编辑
          |
    碰撞、连续性与时序校验
          |
    任务编排与执行
          |
    arm_sdk 网关 + HandClient
```

PC 端用于建模、规划、示教、编辑和调试；Jetson 端负责加载固定版本的轨迹包并执行任务。离线预览不经过硬件输出通路。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/g1_mode6_description/` | G1 模型适配、碰撞包络、工具坐标与 RViz 显示 |
| `src/g1_kdl_ik/` | KDL 逆运动学、双臂 IK、可达性和姿态能力图 |
| `src/g1_upper_body_moveit_config/` | MoveIt 配置、单臂规划、状态/轨迹校验服务 |
| `src/g1_unitree_control/` | LowState 桥接、shadow 执行、arm_sdk 控制网关 |
| `src/g1_tcp_diagnostics/` | TCP、RGB-D 和相机外参诊断 |
| `scripts/` | 任务面板、示教录制、轨迹编辑、优化、分析及测试 |
| `config/` | 规划参数、手型和部署配置 |
| `calibration/` | 历史负载拟合与标定参数 |
| `assets/` | 工作台、灵巧手、碰撞模型和 RViz 资源 |
| `deployment/rail_jetson/` | Jetson 执行器、任务协议、发布与部署工具 |
| `integrations/` | 外部 SDK2 自定义程序和环境脚本的源码快照 |
| `third_party/` | 灵巧手与视觉服务源码 |
| `docs/` | 环境说明、历史交接与成果报告 |

## 环境与运行

开发环境为 Ubuntu 22.04 / ROS 2 Humble，主要使用 C++17 和 Python 3.10。历史机载环境为 ARM64 Ubuntu 20.04 / ROS 2 Foxy。

完整环境配置见 [SETUP.md](docs/SETUP.md)，外部源码版本见 [dependencies.repos](dependencies.repos)。旧代码仍有 `/home/syf/...` 绝对路径，换机器时需要核对模型、SDK、配置和数据路径。

克隆后可先运行不依赖 ROS 和机器人数据的基础测试，仅需 Python 和 NumPy：

```bash
git clone https://github.com/syfalter/g1.git g1_upper_body_ws
cd g1_upper_body_ws
bash scripts/run_onboarding_checks.sh
```

完成 ROS 依赖和路径配置后，在工作区根目录编译：

```bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

常用入口如下，具体环境要求和参数见对应文档：

| 入口 | 用途 |
| --- | --- |
| `ros2 launch g1_mode6_description display.launch.py` | 本体模型显示 |
| `ros2 launch g1_kdl_ik kdl_ik.launch.py run_self_test:=true` | IK 自检 |
| `ros2 launch g1_upper_body_moveit_config sim_moveit.launch.py` | 离线规划示例 |
| `bash scripts/start_task_panel.sh --offline` | 离线轨迹编辑 |
| `bash scripts/stop_task_panel.sh --offline` | 关闭离线编辑器 |
| `ros2 launch g1_unitree_control arm_state_bridge.launch.py` | 真实关节状态只读桥接 |

## 控制接口

`/lowstate` 提供关节、IMU 和电机状态；`/arm_sdk` 为上肢控制输出。网关默认关闭硬件输出，启用后仍需独立 ARM 和状态检查。左右臂分别对应电机 15..21 和 22..28，当前配置使用腰 yaw 12，`motor_cmd[29].q` 是上肢混合权重。

`/plan_single_arm` 和 `/move_single_arm` 使用 `PlanSingleArm.srv`，支持 `LINEAR`、`OMPL`、`AUTO`；`execute:false` 只规划。灵巧手通过 HandClient 访问唯一的 `hand_service`，归一化目标 `q=0` 为闭合、`q=1` 为张开。

详细说明见 [控制包](src/g1_unitree_control/README.md)、[MoveIt 包](src/g1_upper_body_moveit_config/README.md) 和 [IK 包](src/g1_kdl_ik/README.md)。

## 已有工作与文档

| 内容 | 文档 |
| --- | --- |
| 工作台建模与离线预演 | [WORKBENCH_APPLICATION.md](WORKBENCH_APPLICATION.md) |
| D 环视觉规划与任务调试 | [REAL_TASK_APPLICATION.md](REAL_TASK_APPLICATION.md)、[方法记录](D_RING_METHOD_RESEARCH.md) |
| TCP 与相机标定 | [TCP A](scripts/TCP_A_DIAGNOSTICS.md)、[Camera B](scripts/CAMERA_B_DIAGNOSTICS.md)、[Camera C](scripts/CAMERA_C_DIAGNOSTICS.md) |
| 示教任务 | [滑轨](scripts/RAIL_TEACH.md)、[起落架](LANDING_GEAR_TEACH.md)、[屏幕](scripts/SCREEN_TEACH.md) |
| 轨迹编辑与重定时 | [编辑面板](TRAJECTORY_EDITOR_GUIDE.md)、[重定时设计](TRAJECTORY_RETIMING_DESIGN.md) |
| Jetson 执行与发布 | [部署说明](deployment/rail_jetson/README.md) |
| 轨迹优化交接 | [交接记录](docs/trajectory_only_handoff_20260929.md)、[结果](docs/trajectory_only_results_20260929.md) |
| 整体成果 | [2026-10-13 技术报告](docs/report_20261013/G1机器人复杂操作任务技术成果报告_20261013.md) |

历史文档中的“当前状态”和参数属于各自记录日期。2026-09-30 的轨迹交接包含后续版本修订，阅读时以顶部更新为准。`deployment/rail_jetson/` 是本地开发版本，不保证与机载实际部署一致。

## 数据与注意事项

- Git 收录源码、配置、模型和文档，不包含约 108 GB 的 `data/`、运行日志、虚拟环境、构建产物、原始标定 CSV 和视觉权重。数据位置见 [data/README.md](data/README.md)，本机原文件保留。
- `start_task_panel.sh` 不带 `--offline` 会启动真机控制栈。真实运行前需确认控制权、状态反馈、负载参数和现场条件，避免与 Jetson 或其他 SDK 程序同时输出。
- 任务 STOP 可能继续维持支撑，不等同于失能或断电。规划/离线验证通过也不代表接触安全已验证。
- 左臂负载文件标注为右侧参数复制，尚不是独立左臂标定结果。
- 第三方来源、版本和许可说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。
