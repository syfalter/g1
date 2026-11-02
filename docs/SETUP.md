# 环境配置

## 依赖

| 部分 | 依赖 |
| --- | --- |
| 基础测试 | Python 3.10+、NumPy |
| ROS 工作区 | Ubuntu 22.04、ROS 2 Humble、MoveIt 2、KDL、RViz、Qt5、CycloneDDS |
| Python 任务界面 | 系统 Python、PyQt5、NumPy、SciPy、PyYAML、OpenCV、Matplotlib |
| 本体模型 | `unitreerobotics/unitree_ros` 中的 G1 URDF 和 meshes |
| 消息 | `unitreerobotics/unitree_ros2` 中的 `unitree_hg`、`unitree_api`、`unitree_go` |
| 手部与直接 DDS 工具 | Unitree SDK2、`third_party/g1-rh56dfx-control` |
| 视觉识别 | [视觉 requirements](../third_party/g1-rh56dfx-control/recognize/pedal_center_package_v1/requirements.txt)、另行取得的权重 |

外部仓库 commit 固定在 [dependencies.repos](../dependencies.repos)。这些是本机开发时使用的版本，并非最新版本。

## 基础测试

系统已安装 NumPy 时：

```bash
bash scripts/run_onboarding_checks.sh
```

也可以使用独立虚拟环境：

```bash
python3.10 -m venv .venv-onboarding
.venv-onboarding/bin/python -m pip install -r requirements-onboarding.txt
G1_TEST_PYTHON="$PWD/.venv-onboarding/bin/python" bash scripts/run_onboarding_checks.sh
```

该脚本运行 64 项合成数据测试，覆盖轨迹插值、连续限值、状态时效、任务取消和归档校验；不连接 ROS、DDS、SSH 或机器人。`requirements-onboarding.txt` 只用于这组测试，不是完整运行环境。

## 路径布局

历史环境使用以下目录：

```text
/home/syf/
  g1_upper_body_ws/
  unitree_sdk2/
  unitree_ws/
    env/setup_unitree_humble_g1.sh
    unitree_ros/
    unitree_ros2/
```

可以使用 vcstool 将锁定依赖导入新的空目录：

```bash
mkdir -p "$HOME/g1-deps"
vcs import "$HOME/g1-deps" < dependencies.repos
```

导入目录与历史布局不同，需要调整旧代码中的模型、SDK、配置和数据路径。只有基础测试可以直接在任意克隆目录运行；完整 ROS 控制栈尚未完成路径参数化。可用以下命令定位相关设置：

```bash
rg -n '/home/syf|/home/unitree|192\.168\.' src scripts config integrations deployment
```

SDK2 自定义示教程序、构建补丁和历史环境助手保存在 [integrations/](../integrations/README.md)。环境助手按原 `unitree_ws/env` 布局解析路径，不应在快照目录直接 source。

## 编译

先安装 ROS 2 Humble 及上述依赖。在新建的 `unitree_ros2/cyclonedds_ws` 中构建消息包：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/unitree --packages-select unitree_go unitree_hg unitree_api --symlink-install
source install/setup.bash
```

回到本仓库根目录，检查消息环境并安装/构建本地依赖：

```bash
ros2 pkg prefix unitree_hg
ros2 pkg prefix unitree_api
rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys="unitree_hg unitree_api unitree_go" -r -y
colcon build --base-paths src --symlink-install
source install/setup.bash
```

`--base-paths src` 限定五个本地 ROS 包，避免递归构建 `assets/` 和 `integrations/` 的独立源码。ROS 运行使用系统 Python；不要与不同版本的 Conda Python 混用。

`scripts/build_task_hand_client.sh` 编译手部客户端，`scripts/build_task_ompl_route.sh` 编译本地 OMPL Python 扩展。脚本中的 SDK/ROS 路径及架构需与目标机器一致，预编译 `.so` 不随 Git 上传。

## 离线运行

source ROS 与工作区 install 后，设置隔离域：

```bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER G1_DDS_INTERFACE_ADDRESS
```

本体显示支持指定模型路径：

```bash
ros2 launch g1_mode6_description display.launch.py model_path:="$HOME/g1-deps/unitree_ws/unitree_ros/robots/g1_description/g1_29dof_lock_waist_rev_1_0.urdf"
```

IK 和 MoveIt 入口仍需先适配历史路径，分别运行：

```bash
ros2 launch g1_kdl_ik kdl_ik.launch.py run_self_test:=true
```

```bash
ros2 launch g1_upper_body_moveit_config sim_moveit.launch.py
```

离线编辑器使用用户级 systemd：

```bash
bash scripts/start_task_panel.sh --offline
```

使用完毕后关闭：

```bash
bash scripts/stop_task_panel.sh --offline
```

无 systemd user session 时可前台运行 `bash scripts/task_editor_offline.sh run`，用 Ctrl+C 退出。GUI 需要显示服务器；历史轨迹需单独取得，见 [数据说明](../data/README.md)。

## 测试范围

`scripts/test_*.py` 中的完整测试还依赖 ROS、Qt、外部模型和历史数据，基础测试脚本仅运行固定子集。本次仓库整理不包含新环境全量构建、相机采集或真机验收。
