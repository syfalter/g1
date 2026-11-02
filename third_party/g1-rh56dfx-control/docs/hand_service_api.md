# RH56 手部服务接口使用文档

## 1. 服务说明

`hand_service` 是左右 RH56 串口的唯一硬件访问进程，对外提供：

- 兼容宇树 Inspire 接口的 DDS 位置命令和状态 Topic；
- 支持单指控制、故障清除、状态查询、永久参数和注册动作管理/执行的 Unitree RPC；
- 头文件形式的 C++ 客户端 `src/rh56/hand_client.hpp`。

位置统一使用归一化值：`0` 表示闭合，`1` 表示张开。单手关节顺序与
掩码定义如下：

| 索引 | 关节 | 掩码位 |
|---:|---|---:|
| 0 | pinky | `0x01` |
| 1 | ring | `0x02` |
| 2 | middle | `0x04` |
| 3 | index | `0x08` |
| 4 | thumb-bend | `0x10` |
| 5 | thumb-rotation | `0x20` |

DDS 中的 12 个关节按右手 6 个关节、左手 6 个关节排列。

## 2. 编译与启动

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target hand_service hand_clear_fault -j2
```

启动服务会取得硬件控制权。启动前必须确认机器人稳定、手部工作空间无障碍，
并且可以立即操作物理急停：

```bash
./build/src/hand_service --execute --hand both --network eth0
```

| 参数 | 说明 |
|---|---|
| `--execute` | 必填，明确允许访问硬件 |
| `--hand left\|right\|both` | 启用的手，默认 `both` |
| `--network INTERFACE` | DDS 网卡；不填写时使用 SDK 默认值 |
| `--poses-file PATH` | 注册动作文件；默认 `data/hand_poses.json` |

服务会写入 `run/hand_service.pid`，并独占所选手的串口。如果本工程的其他
控制进程已经占用串口，服务将拒绝启动。

正常停止使用 `Ctrl+C` 或 `SIGTERM`；服务退出前会请求已启用的手保持当前位置。
软件保持不能替代断电或机器人物理急停。

## 3. DDS 连续控制接口

### 3.1 位置命令

- Topic：`rt/inspire/cmd`
- 类型：`unitree_go::msg::dds_::MotorCmds_`
- 电机数组长度：至少 12
- 有效字段：`cmds[i].q`
- 数值范围：`[0, 1]`

发布间隔必须小于 300 ms。长度不足或包含非法位置值的消息会被丢弃。连续命令
超过 300 ms 未更新时，服务读取当前反馈位置，并发送一次当前位置保持命令。

```cpp
unitree_go::msg::dds_::MotorCmds_ command;
command.cmds().resize(12);
for (std::size_t i = 0; i < 12; ++i)
    command.cmds()[i].q(target[i]);
publisher->Write(command);
```

### 3.2 位置状态

- Topic：`rt/inspire/state`
- 类型：`unitree_go::msg::dds_::MotorStates_`
- 电机数组长度：12

| DDS 字段 | RH56 数据 |
|---|---|
| `q` | 归一化执行器位置反馈 |
| `mode` | RH56 电机状态码 |
| `temperature` | 摄氏温度 |
| `lost` | 累计遥测刷新失败次数 |
| `reserve[0]` | RH56 错误位 |
| `reserve[1]` | RH56 电机状态码 |

力、当前电流、目标位置以及完整的一致性诊断快照应通过状态 RPC 获取。

## 4. RPC 接口

- 服务名：`rh56_hand`
- API 版本：`1.3.0.0`

| API ID | 操作 |
|---:|---|
| 1001 | 设置带关节掩码的位置目标 |
| 1002 | 清除故障 |
| 1003 | 查询单手完整状态 |
| 1004 | 设置抓握参数并发送目标 |
| 1005 | 保持当前位置 |
| 1006 | 注册动作管理与执行 |
| 1007 | 查询或修改永久参数 |

Unitree SDK 使用 JSON 字符串承载 RPC 的请求和响应。C++ 上层应直接使用
`rh56::HandClient`，不要自行发布 SDK 内部的 RPC Topic。

### 4.1 SetTargets，API 1001

请求：

```json
{
  "hand": "right",
  "joint_mask": 4,
  "q": [0.0, 0.0, 0.75, 0.0, 0.0, 0.0],
  "command_id": 42,
  "timeout_ms": 300
}
```

- `hand` 只能是 `right` 或 `left`；
- `q` 始终包含 6 个值，只使用 `joint_mask` 选中的位置；
- `joint_mask` 有效范围为 `1..63`；
- `timeout_ms` 有效范围为 `50..5000`；
- 需要持续控制时，应在 `timeout_ms` 到达前发送下一条命令；
- 超时后服务将目标改为当时的反馈位置；
- 如果该手正在接收 DDS 位置流，RPC 返回 `102 BUSY`。

操作响应：

```json
{
  "code": 0,
  "message": "ok",
  "request_id": 42,
  "affected_mask": 4,
  "error": [0, 0, 0, 0, 0, 0],
  "status": [1, 1, 1, 1, 1, 1],
  "temperature": [31, 30, 32, 31, 29, 30]
}
```

### 4.2 ClearFault，API 1002

请求：

```json
{
  "hand": "right",
  "joint_mask": 4,
  "request_id": 43
}
```

服务首先取消该手尚未执行的控制命令，然后按以下顺序操作：

1. 读取位置、错误、状态和温度；
2. 将所有故障执行器的目标设置为当前位置；
3. 需要时写入 `CLEAR_ERROR(0x03EC)=1`；
4. 再次发送当前位置保持；
5. 读取错误和状态寄存器验证结果。

`CLEAR_ERROR` 是整只手共用的固件寄存器。`joint_mask` 指定需要检查和验证的关节，
但一次写入可能同时清除同一只手上的其他可清除故障。过温故障不会执行清除，必须
等待温度下降并由固件自动恢复。

响应与 SetTargets 使用相同的操作响应结构。`affected_mask` 表示本次请求实际处理
的关节；如果请求的关节没有故障，返回成功且 `affected_mask` 为 0。

命令行客户端也通过该 RPC 工作，运行前必须先启动 `hand_service`：

```bash
./build/src/hand_clear_fault \
  --hand right --joint middle --network eth0 --execute
```

### 4.3 GetState，API 1003

请求：

```json
{"hand":"left","refresh":true}
```

响应：

```json
{
  "code": 0,
  "message": "ok",
  "hand": "left",
  "online": true,
  "feedback_q": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
  "target_q": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
  "force": [0, 0, 0, 0, 0, 0],
  "current": [0, 0, 0, 0, 0, 0],
  "force_limit": [250, 250, 250, 250, 250, 250],
  "current_limit": [180, 180, 180, 180, 180, 180],
  "contact": [false, false, false, true, false, false],
  "contact_monitoring": true,
  "error": [0, 0, 0, 0, 0, 0],
  "status": [1, 1, 1, 1, 1, 1],
  "temperature": [31, 30, 32, 31, 29, 30],
  "lost_count": 0,
  "last_command_id": 42,
  "timestamp_ms": 1788336000000
}
```

`feedback_q` 是执行器反馈，不是外部测量的指节角度。手指被动机构释放后，其可见
姿态还会受重力、手掌方向和接触状态影响。

`refresh=false` 返回服务缓存，适合高频状态展示；`refresh=true` 会在返回前读取位置、
力、电流、限制值、错误、状态和温度，适合诊断和柔顺示教。

`contact` 是按六关节顺序排列的空闲接触判定。只有 `contact_monitoring=true` 时该数组
才有效；存在 DDS 或 RPC 动作控制时，服务会将整只手的监测暂停并把 `contact` 清为
`false`，避免动作造成的力波动被误判。手重新空闲后，服务以 10 Hz 读取力传感器，先用
三次采样建立基线，然后在当前值与基线的绝对差达到永久参数 `contact_threshold` 时置位。
差值回落到阈值一半以下时清除接触；未接触期间基线会缓慢跟踪静态漂移。

### 4.4 ApplyGrip，API 1004

```json
{
  "hand": "right",
  "q": [0.2, 0.2, 0.2, 0.2, 0.3, 0.4],
  "force_grams": 250,
  "current_ma": 180,
  "command_id": 44,
  "timeout_ms": 5000
}
```

服务会先检查实时电流、温度和错误，再依次写入电流限制、力阈值和位置目标。
`current_ma` 的服务安全上限为 300 mA；响应使用通用操作响应结构。

### 4.5 Hold，API 1005

```json
{"hand":"right","request_id":45}
```

服务读取反馈位置并将它设置为新目标，同时取消该手尚未执行的控制命令。正常退出
`hand_service` 时也会对所有已启用的手执行同样的保持操作。

### 4.6 Settings，API 1007

永久参数由常驻的 `hand_service` 管理，默认写入 `data/hand_settings.json`，因此生产环境
不需要启动 Web。读取请求：

```json
{"write":false,"request_id":46}
```

修改时必须提交完整配置：

```json
{
  "write": true,
  "request_id": 47,
  "settings": {
    "right_grip_force_grams": 300,
    "right_grip_current_ma": 200,
    "left_grip_force_grams": 300,
    "left_grip_current_ma": 200,
    "contact_threshold": 50
  }
}
```

抓握力范围为 `50..1000 g`，电流范围为 `50..300 mA`，接触波动阈值范围为
`1..1000`（RH56 力寄存器原始单位）。成功响应的 `settings` 返回当前完整配置。Web 中
调整任一抓握滑块或接触阈值后，会通过该 RPC 自动保存。

## 5. C++ 客户端用法

```cpp
#include "rh56/hand_client.hpp"
#include <unitree/robot/channel/channel_factory.hpp>

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

rh56::HandClient hand;
hand.SetTimeout(1.0f);
hand.Init();

rh56::StateReply state;
int32_t result = hand.GetState("right", state);  // 空闲接触状态由服务后台持续更新

if (result == 0 && state.contact_monitoring && state.contact[3])
    std::cout << "右手食指检测到空闲接触\n";

rh56::SetTargetsRequest command;
command.hand = "right";
command.joint_mask = 1u << 2;  // middle finger
command.q = {0.0f, 0.0f, 0.75f, 0.0f, 0.0f, 0.0f};
command.command_id = 42;
command.timeout_ms = 300;

rh56::OperationReply reply;
result = hand.SetTargets(command, reply);
if (result != 0)
    std::cerr << reply.message << '\n';
```

上层程序的 CMake 配置：

```cmake
find_package(unitree_sdk2 REQUIRED)
target_include_directories(my_controller PRIVATE /path/to/workspace/src)
target_link_libraries(my_controller PRIVATE unitree_sdk2)
```

清故障示例：

```cpp
rh56::ClearFaultRequest request;
request.hand = "right";
request.joint_mask = 1u << 2;
request.request_id = 43;

rh56::OperationReply reply;
int32_t result = hand.ClearFault(request, reply);
```

永久参数示例：

```cpp
rh56::SettingsMessage settings;
int32_t result = hand.GetSettings(settings);

rh56::HandSettings updated = settings.settings;
updated.contact_threshold = 75;
result = hand.SetSettings(updated, 47, settings);
```

## 6. 结果码

| 数值 | 名称 | 含义 |
|---:|---|---|
| 0 | OK | 操作完成 |
| 100 | INVALID_ARGUMENT | 手、掩码、位置或超时参数非法 |
| 101 | HAND_UNAVAILABLE | 服务启动时未启用该手 |
| 102 | BUSY | 其他命令源正在控制该手 |
| 103 | POSE_NOT_FOUND | 注册动作 ID 不存在 |
| 200 | SERIAL_TIMEOUT | 未收到必要的串口遥测 |
| 201 | WRITE_REJECTED | 固件未确认写命令 |
| 202 | INVALID_RESPONSE | 客户端无法解析 RPC 响应 |
| 203 | STORAGE_ERROR | 注册动作或永久参数文件读写失败 |
| 300 | JOINT_FAULTED | 所选关节存在故障，拒绝运动 |
| 301 | OVER_TEMPERATURE | 过温故障必须冷却后自动恢复 |
| 302 | FAULT_REMAINS | 清除后错误或故障停止状态仍存在 |
| 303 | STALE_COMMAND | 命令已经超过允许时效 |

Unitree SDK 自身的传输错误码由 `HandClient` 原样返回。

## 7. 控制权与使用约束

- 每只手只使用一个位置命令源；
- 活跃的 DDS 位置流优先于新发起的 RPC 位置控制；
- 成功的 RPC 位置命令持有该手控制权，直到 `timeout_ms` 到期；
- 清故障会取消该手的待执行命令，并临时阻止 DDS 位置命令覆盖恢复过程；
- 左右手的硬件事务分别加锁，所有串口请求都经过同一事务实现；
- 网页、柔顺示教、故障清除和软件保持工具均为 `HandClient` 客户端，可以与服务
  同时运行，但每只手仍只应有一个位置命令源；
- `hand_service` 运行时不要启动官方 Inspire 串口服务或其他直接串口程序。

## 8. 内置客户端启动方式

```bash
# 网页控制（先启动 hand_service）
./build/src/hand_web_control --hand both --network eth0

# 柔顺示教
./build/src/hand_compliant_teach \
  --execute --hand both --joint all --profile slow --network eth0

# 保持当前位置
./build/src/hand_hold_position both --network eth0
```

## 9. 注册动作接口

注册动作由常驻的 `hand_service` 管理，默认保存在 `data/hand_poses.json`。Web 只是可选
调试客户端，它的保存、改名、延时、删除和执行按钮都调用以下 RPC。关闭 Web 后，
上层仍可完整使用动作接口。

API 1006 统一使用 `PoseRequest`，通过 `action` 区分操作：

| `action` | 必需字段 | 作用 |
|---|---|---|
| `list` | 无 | 查询全部动作 |
| `save` | `name`、`right`、`left` | 保存动作，`delays_ms` 省略时为全零 |
| `rename` | `id`、`name` | 改名 |
| `set_delays` | `id`、`delays_ms` | 修改六个关节启动延时 |
| `delete` | `id` | 删除动作 |
| `execute` | `id`、`hand` | 执行动作 |

动作结构：

```json
{
  "id": "1788336000000-1",
  "created": 1788336000000,
  "name": "拇指延后抓取",
  "right": [0.2, 0.2, 0.2, 0.3, 0.4, 0.5],
  "left": [0.2, 0.2, 0.2, 0.3, 0.4, 0.5],
  "delays_ms": [0, 0, 0, 0, 300, 300]
}
```

`delays_ms` 按小指、无名指、中指、食指、拇指弯曲、拇指旋转排列，单位为毫秒；
左右手同名关节共用延时，允许范围为 `0..3000`。

所有操作返回 `PoseReply`：查询结果在 `poses`，单条修改或执行结果在 `pose`。保存时
`id` 留空由服务生成，传入已存在 ID 会直接返回原动作，便于调用方安全重试。修改立即
持久化；不存在的 ID 返回 `103 POSE_NOT_FOUND`。

### 9.1 执行语义

```json
{
  "action": "execute",
  "id": "1788336000000-1",
  "hand": "both",
  "request_id": 104,
  "timeout_ms": 5000
}
```

- `hand` 可为 `right`、`left` 或 `both`；`both` 表示服务启动时已启用的所有手；
- 服务先读取并保持当前状态，再按 `delays_ms` 将相同延时的关节合并为一个掩码发送；
- 活跃 DDS 位置流或另一个 RPC 控制操作存在时返回 `102 BUSY`；
- `timeout_ms` 从最后一组关节命令发出后开始计算，到期后保持当时位置；
- 响应在最后一组延时命令发出后返回，不代表机械运动已经到达目标；
- 任一步骤失败时，已经进入本次动作控制的手会保持当前位置；
- `Hold` RPC 可以中断正在等待启动延时的动作。

成功响应的 `duration_ms` 是最后一个关节的启动延时，`affected_hands` 是实际执行的手。

### 9.2 上层直接调用示例

生产环境只需要启动 `hand_service`。上层不启动 `hand_web_control`，也不使用 Web token：

```cpp
#include "rh56/hand_client.hpp"
#include <unitree/robot/channel/channel_factory.hpp>

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

rh56::HandClient hand;
hand.SetTimeout(8.0f);  // 覆盖最长 3 s 启动延时和 RPC 开销
hand.Init();

rh56::PoseReply list;
if (hand.ListPoses(list) != 0)
    throw std::runtime_error(list.message);

rh56::PoseReply reply;
const std::string id = list.poses.at(0).id;  // 实际程序应保存稳定 ID
if (hand.ExecutePose(id, "both", 104, reply) != 0)
    throw std::runtime_error(reply.message);
```

Web 的 `/api/poses/*` HTTP 路径继续保留用于调试，但它们只是上述 RPC 的转发层，
不应作为生产上层的依赖。

## 10. 调用方建议

- DDS 连续控制建议以 20 至 50 Hz 发布；
- RPC 连续控制的发送周期应小于 `timeout_ms` 的一半；
- 每条命令使用单调递增的 `command_id`；
- 上层动作完成条件应结合 `feedback_q`、错误位和超时，而不是只检查 RPC 返回成功；
- 收到 `300`、`301` 或 `302` 后应停止该关节的动作序列，并进入故障处理状态。
