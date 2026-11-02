# 工作区外的本地扩展

本目录保存 2026-11-02 检查到的相关本地代码，方便理解历史遥控示教和负载辨识；没有修改原 SDK2 工作区，没有打包它的二进制或全量上游源码。

## SDK2 扩展

基线：`unitreerobotics/unitree_sdk2` commit `9754cd153af3da471b0fe5f3aa535e426fb11db3`。

`unitree_sdk2/example/g1/high_level/` 保存六个本地新增 `.cpp`：

| 程序 | 用途 | 注意 |
| --- | --- | --- |
| `g1_arm_readonly_diagnostics` | 读取关节/服务/模式诊断 | 仍需要核对连接环境 |
| `g1_remote_key_check` | 遥控按键映射观察 | 不代表新固件的映射已经验证 |
| `g1_arm_writer_match_probe` | DDS writer 匹配诊断 | 会创建 writer，不作为新人只读入口 |
| `g1_arm_sdk_safe_probe` | 历史受保护接管试验 | 可输出控制命令 |
| `g1_arm_sdk_clear_weight` | 历史权重清理工具 | 会改变控制权/支撑，不是常规退出方法 |
| `g1_right_arm_payload_calibration` | 负载标定、保持、导纳、遥控示教 | 同一源码另编译左侧目标；属于真实控制程序 |

`g1_right_arm_payload_calibration.cpp` 直接复用同目录 `g1_arm_sdk_safe_probe.cpp`，不能只复制一个文件。`g1-cmake.patch` 保存原 SDK2 的 CMake 差异，构建目标还包括 `g1_left_arm_payload_calibration`。

### 在新的 SDK2 副本中恢复

先按 [dependencies.repos](../dependencies.repos) 获取基线；只对确认干净的副本操作，不覆盖已有开发目录。`G1_WS` 和 `SDK2` 改成自己的实际绝对路径。

```bash
export G1_WS=/absolute/path/to/g1_upper_body_ws
export SDK2=/absolute/path/to/unitree_sdk2
git -C "$SDK2" apply --check "$G1_WS/integrations/unitree_sdk2/g1-cmake.patch"
cp -n "$G1_WS"/integrations/unitree_sdk2/example/g1/high_level/*.cpp "$SDK2/example/g1/high_level/"
git -C "$SDK2" apply "$G1_WS/integrations/unitree_sdk2/g1-cmake.patch"
source /opt/ros/humble/setup.bash
cmake -S "$SDK2" -B "$SDK2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SDK2/build" --target g1_right_arm_payload_calibration g1_left_arm_payload_calibration g1_remote_key_check -j2
```

`cp -n` 只适合新副本，已有同名文件时应停止并人工比较，不能认为跳过复制后的程序仍与快照一致。编译需要 Eigen3、orocos-kdl、kdl_parser 和 SDK 自身依赖。构建不意味着允许运行；源码中历史模型路径也需核查。

## 环境助手快照

`env/setup_unitree_humble_g1.sh` 原来位于 `/home/syf/unitree_ws/env/`，通过相对路径找到 Unitree 工作区，设置真实 DDS 环境。它依赖已构建的 `cyclonedds_ws/install/setup.bash` 与 `unitree_ros2/install/setup.bash`，不是包管理器或自动安装脚本。

只有在理解原布局并完成构建后，才把经过现场网卡核查的副本放回相应 `unitree_ws/env/`。不要直接在本 `integrations/env` 目录 source，不要用它启动离线练习。

历史本地 DDS 源码基线另记为：CycloneDDS `5041f3560c088c99e5088b2b8520b69169621196`，rmw_cyclonedds `fa8831b9f331d669c3ea408fd00606a223c6bbd9`。它们未随源码快照收录；是否复用必须根据目标 ROS/SDK ABI 决定。
