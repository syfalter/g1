# 回正轮盘识别部署包

此目录是可直接交给其他模块使用的两阶段 YOLOv8 推理包。它包含模型权重和端到端接口，不包含训练数据、标注工具或训练脚本。

模型分工：

- `models/stage1_best.pt`：整套回正轮盘的外框（`pedal_centered`）。
- `models/stage2_best.pt`：外框内部的黑色圆盘框（`black_center`）。

端到端结果还会计算黑色圆盘中心、倒梯形手指操作区、梯形对角线交点，以及沿圆心—交点半径向外偏移后的最终操作点。

## 安装

使用训练时相同的 Python 环境（推荐 CUDA 可用的 PyTorch 环境）：

```powershell
cd D:\recognition\pedal_center_package_v1
pip install -r requirements.txt
```

权重默认从当前目录下的 `models` 自动加载；无需传入当前工程中 `runs` 目录的路径。

## Python 接口

```python
import cv2
from pedal_center_recognizer import PedalCenterRecognizer

# 应用启动时创建一次，后续图像重复调用 predict。
recognizer = PedalCenterRecognizer(device="0")  # 无 GPU 时改为 device="cpu"

image_bgr = cv2.imread(r"D:\path\to\frame.png")
result = recognizer.predict(image_bgr)
payload = result.to_dict()

if payload["ready"]:
    operation_point = payload["operation_point"]
    black_circle_center = payload["black_center_point"]
    print(operation_point, black_circle_center)
```

`payload` 为可直接 JSON 序列化的字典，字段如下：

```text
ready                              # 两阶段均成功且计算出操作点时为 true
pedal_centered                     # [x1, y1, x2, y2]，完整轮盘外框
black_center                       # [x1, y1, x2, y2]，黑色圆盘框
black_center_point                 # 黑框对角线交点
operation_zone                     # [[x,y], ...]，上宽下窄的倒梯形
operation_diagonal_intersection    # 倒梯形对角线交点
operation_point                    # 最终供手指使用的点，沿半径远离圆心
stage1_confidence
stage2_confidence
```

## RGB-D 三维关键点

二维像素点加相机内参只能确定一条空间射线；要获得米制三维点，必须使用与彩色图像同一时刻、并且已经对齐到彩色图像的深度帧。`captures/` 中现有的 JPEG 没有深度数据，不能事后可靠地转换为绝对 3D。

生产环境由项目根目录下的 `vision_service` 常驻打开 D435I 和加载模型，但只在
`RecognizeOnce` 请求到达时执行一次推理。服务接口见
[`../../docs/vision_service_api.md`](../../docs/vision_service_api.md)。本目录仍保留单次命令行
适配器用于离线验证，它与服务共用 `PedalVisionRuntime`，没有重复采集逻辑：

```bash
cd recognize/pedal_center_package_v1
/home/unitree/.miniconda3/envs/Unitree/bin/python \
  realsense_pedal_3d.py --device 0
```

默认头部 D435I 序列号为 `243322073672`，分辨率为 `1280x720@30`；可通过 `--serial`、`--width`、`--height` 和 `--fps` 修改。该相机的 RGB 流不能同时被 `videohub` 和本程序占用；若启动时报设备忙，需要先正常停止占用头部 RGB Camera 的服务。

输出中的 `points_camera.operation_point` 为 `[x, y, z]`（米），坐标系是
`head_color_camera_optical_frame`：`+x` 图像向右、`+y` 向下、`+z` 向前。单次命令会在
项目 `captures/` 下生成同一 ID 的四个文件；服务仅在请求设置 `save_artifacts=true` 时保存：

- `*_color.png`：无损 BGR8 彩色帧；
- `*_depth.png`：对齐到彩色图的 16 位 Z16 深度帧；
- `*_annotated.jpg`：二维识别标注；
- `*_result.json`：2D/3D 结果、帧号、时间戳、编码、深度比例、设备型号/序列号，以及彩深内外参。

### 转换到机器人目标坐标

RealSense 可以提供相机内参以及彩色/深度传感器之间的外参，但无法测出相机在机器人上的安装位姿。可以提供经过标定的 `head_color_camera_optical_frame -> target_frame` 刚体变换；程序不会使用猜测值。外参 JSON 格式如下：

```json
{
  "source_frame": "head_color_camera_optical_frame",
  "target_frame": "base_link",
  "rotation": [["r00", "r01", "r02"], ["r10", "r11", "r12"], ["r20", "r21", "r22"]],
  "translation_m": ["tx", "ty", "tz"]
}
```

以上字符串是字段占位符，实际文件必须替换为标定得到的数值。其中
`p_target = rotation * p_camera + translation_m`。程序会拒绝非正交、行列式不为 `+1`
或源坐标系名称不匹配的配置。使用已标定外参运行：

```bash
/home/unitree/.miniconda3/envs/Unitree/bin/python \
  realsense_pedal_3d.py --device 0 \
  --target-extrinsics /path/to/camera_to_base.json
```

提供后，结果写入 `points_target` 并标记具体 `target_frame`。未提供时
`points_target` 为空且 `transform_status` 为 `not_configured`；相机坐标结果仍然有效。

底层调用接口可嵌入现有程序：

```python
from pedal_3d_projector import CameraIntrinsics, project_pedal_result

# color_frame 与 depth_frame 必须来自同一帧，且 depth 已 rs.align(rs.stream.color)。
intrinsics = CameraIntrinsics(fx, fy, cx, cy, color_width, color_height, distortion)
result_2d = recognizer.predict(color_bgr)
result_3d = project_pedal_result(
    result_2d, aligned_depth_raw, intrinsics,
    depth_scale=depth_scale_m_per_unit, patch_radius=2,
)
if result_3d.ready:
    operation_point_camera_m = result_3d.points_camera["operation_point"]
```

三维结果不是机器人基座或手腕坐标。用于运动前，还必须使用独立标定得到的相机到目标坐标系外参进行变换，并完成现场安全验证。

可视化复核图像：

```python
review_bgr = recognizer.annotate(image_bgr, result)
cv2.imwrite(r"D:\path\to\review.png", review_bgr)
```

## 命令行批量推理

```powershell
python .\infer_pedal_center_twostage.py `
  --source D:\recognition\test_data `
  --output D:\recognition\pedal_inference_output
```

`--source` 可以是一张 `.png/.jpg/.jpeg` 图片，或一个非递归图片目录。输出目录中会生成每张图片的可视化结果和 `predictions.json`。

默认阈值已针对当前头部 RealSense D435I 数据设定。除非出现误检或模型重新训练，建议保留默认的 `--stage1-conf 0.10` 和 `--stage2-conf 0.001`。
