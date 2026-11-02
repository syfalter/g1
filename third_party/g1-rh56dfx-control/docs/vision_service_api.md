# 视觉识别服务接口

## 服务说明

`vision_service` 与 `hand_service` 平级，通过 Unitree RPC 向上层提供一次性踏板识别。
服务启动时常驻打开头部 D435I 并加载两阶段 YOLO 模型，但不会循环推理；每次
`RecognizeOnce` 请求只对一组同步 RGB-D 帧执行一次识别。

- 服务名：`vision_service`
- API 版本：`1.0.0.0`
- 相机坐标系：`head_color_camera_optical_frame`
- 三维单位：米

`vision_service` 是 D435I 的 RGB/Depth 所有者，不能与 `videohub_pc4` 同时占用 RGB 流。
部署时必须先调整 `master_service` 的监督配置，避免其重新拉起 `videohub_pc4`。

## 编译与启动

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target vision_service -j2

./build/src/vision_service --network eth0
```

未标定相机安装外参时可直接启动。服务仍返回 `points_camera`，但
`points_target` 为空、`transform_status` 为 `not_configured`。

标定完成后：

```bash
./build/src/vision_service \
  --network eth0 \
  --target-extrinsics config/head_camera_to_base.json
```

外参中的 `target_frame` 不被写死，可以是实际系统采用的 `base_link` 或其他机器人坐标系。

## RecognizeOnce，API 2001

请求：

```json
{
  "request_id": 42,
  "require_3d": true,
  "require_target_frame": false,
  "save_artifacts": false
}
```

- `require_3d`：操作点没有有效深度时返回 `103`。
- `require_target_frame`：没有配置目标坐标外参时返回 `104`。
- `save_artifacts`：保存彩色、16 位深度、标注图和 JSON；默认不写文件。

主要响应字段：

```json
{
  "code": 0,
  "message": "ok",
  "request_id": 42,
  "valid_2d": true,
  "valid_3d": true,
  "target_frame_ready": false,
  "operation_point": [640, 380],
  "points_camera": {
    "operation_point": [0.0053, 0.0780, 0.5360]
  },
  "points_target": {},
  "camera_frame": "head_color_camera_optical_frame",
  "target_frame": "",
  "transform_status": "not_configured",
  "units": "m"
}
```

响应还包含两阶段检测框、置信度、三个二维/三维关键点、无效深度键、帧号、
彩深时间戳差、相机内参和彩深外参。完整类型定义见 `src/vision/vision_api.hpp`。

结果码：

| code | 含义 |
|---:|---|
| 0 | 成功 |
| 100 | 请求参数错误 |
| 101 | Worker 不可用 |
| 102 | 未检测到踏板 |
| 103 | 操作点深度无效 |
| 104 | 请求目标坐标，但外参未配置 |
| 200 | Worker 内部错误 |
| 201 | 响应无法解析 |

## GetStatus，API 2002

请求：

```json
{"request_id":43}
```

返回模型、相机和外参状态。`state` 正常待机时为 `idle`，处理识别请求时为
`capturing`，最近一次采集失败时为 `degraded`。

## C++ 客户端

```cpp
#include "vision/vision_client.hpp"

unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
vision::VisionClient client;
client.SetTimeout(60.0F);
client.Init();

vision::RecognizeRequest request;
request.request_id = 42;
request.require_3d = true;
request.require_target_frame = false;

vision::ObservationReply reply;
const int32_t result = client.RecognizeOnce(request, reply);
if (result == 0 && reply.valid_3d) {
    const auto& point = reply.points_camera.operation_point;
}
```

调用者与服务必须使用相同的 Unitree DDS 网卡。未完成真实外参标定前，不得将
相机坐标直接作为机器人运动目标。
