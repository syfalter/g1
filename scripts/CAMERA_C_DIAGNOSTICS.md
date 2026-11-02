# 实验 C：离线回放与配对门控

更新：2026-09-13。本版不改 TCP、URDF、相机配置，不控制机器人。
**新版本已实现新流取帧 + 静止实际反馈的诊断配对，并接通求解；现场状态见下节。**

## 当前版本：新流静态配对

本节取代下文早期版本的“始终拒绝、不能求解”描述。
每次 C 的显式取图请求携带 `restart_stream=true`，原 vision_service 在采集锁内对同一
pipeline 执行 stop/start，重建图像队列，预热至少 15 帧再取图。不创建第二个 pipeline，
不重启机器人或控制板，不发机械臂/手指命令。B/任务请求默认不启用此选项。

准入需同时满足：RPC 请求 ID 与原图归档一致、owner 新流事件顺序与帧身份正确、
用户本次确认刚性安装与停稳、完整 actual 窗口的来源/模型/状态和运动检查通过、RGB PnP 有效。
通过后 `pairing_valid=true`，fit/validation 样本 `solve_usable=true`；旧快照没有这些证据仍拒绝。
1 mm / 0.3 度、50 ms 缺帧等门限未放宽。结果是静态软件关联，不宣称硬件同步或
DDS 传输年龄已认证，也不自动应用候选。机载 pyrealsense2 为 2.55.1.6486；已核对
对应 SDK v2.55.1 pipeline stop/start 的队列重建实现。

操作：保持原任务面板和导纳；关闭旧 C 窗口后执行 `bash scripts/start_camera_c.sh --live`。
选一侧腕及安装 ID，选择 fit，拖至新姿态并停稳，点击“确认停稳采样”，保持到采样完成。
先确认一条显示“配对合格”；再采多轴姿态。求解至少 6 条合格 fit，还需满足姿态覆盖门限。
独立验证用 validation，不能根据残差好坏调换分组。候选仅保存/导出，不修改 URDF/TCP。

部署备份：机载 `run/camera_c_stream_deploy_20260913/`。Python 与 C++ RPC 请求类型均需更新，
否则旧 C++ 接口会丢弃新参数。需通过正常 `sudo systemctl restart vision_service.service`
加载，不能只检查 Python status；必须用真正 DDS RPC 取图核对 `frame_timing.capture_contract`。
本地编译及正反例检查通过。2026-09-13 用户再次正常重启相机服务后，真实 DDS 新流取图检查通过：
`logs/camera_c_stream_live/3b85cda45eb54b0ab3179c53f8e3015c/`，请求 ID 1789283300607，
新流 ID b90421b7e9204dbfa7f97b73d81eb35d，预热 15 帧，服务端采集约 6.19 秒。
本次仅验证新流取图与下载证据，没有自动加入现场标定会话；首条完整实际姿态配对仍需用户停稳采样检查。

## 2026-09-13 导纳采样更新

- 一侧腕即可。保持原任务 owner 和导纳状态，不需重启任务面板或控制器。
- 在线选择 fit/validation，点击“确认停稳采样（配对待验收）”，逐次确认板刚性固定和机械臂停稳。
  仍检查完整 actual 窗口的腕相对躯干运动：1 mm / 0.3 度上限未放宽。preview 快照不能替代该确认。
- C 专用 `camera_c_actual_reader` 只读接收状态，经本地管道交给 Python，避免旧 Humble
  Python 绑定丢失逐消息来源。旧 CycloneDDS 的 publication handle 通过内置发布者发现样本
  映射为 GUID，再与 ROS 发现列表核对；不把唯一发现的发布者冒充消息来源。保留原始句柄。
- 新归档使用同机 `CLOCK_MONOTONIC` 检查接收窗口，墙钟跳变单独记录。
  owner 新鲜度使用单调接收年龄；owner payload 墙钟只与它自身的接收墙钟比较。
  旧档案仍按旧合同拒绝，不补造 GID 或追认预览样本。
- 关闭并重新启动 C 才加载新接收器和面板；不关闭原任务面板。首次缺少接收器会自动编译。
- 此按钮是诊断采集入口，并非“配对合格”开关。相机服务曝光边界与 DDS 传输年龄仍待验收；
  即使静止检查通过，`pairing_valid` 和 `solve_usable` 仍为 false，勿开始批量正式采样。

## 一键启动

```bash
bash /home/syf/g1_upper_body_ws/scripts/start_camera_c.sh --offline --session /home/syf/g1_upper_body_ws/data/camera_c/sessions/0c1068fcc2b34e228b6489631980d4e2/session_20260912_235934_203648.json
```

这份 C 会话复制自最新 B 的 28 条归档，不是腕部刚性安装样本。保留原始用途、排除和来源审计；
24 条 RGB 可计算、0 条正式配对。不能用这些图估计真实 C 候选。
加 `--rviz` 在 RViz 中打开同一 C 插件；默认独立 Qt 窗口。`--build` 只编译诊断包，
首次没有窗口二进制时脚本也会自动编译。`--backend-only` 不开窗口。

## 窗口操作

1. 加载现有 C JSON，或“导入归档”选择含 `capture.json`、`server_result.json`、
   `manifest.json`、`rgb.png`、`depth_aligned.png` 的完整目录。导入自动保存。
2. 表格选择样本；切换原始 RGB/角点，用图标完整显示、聚焦板面及缩放。拖动画面可平移。
   右侧保留检测、PnP、深度诊断、拒绝原因与原请求审计。
3. 用途明确选择 fit/validation/preview；改变用途需确认，排除/恢复需记录原因。
   不因验证表现差重新分组或删样本。回放历史 B 不会补造 actual。
4. “复算”重新校验 SHA 并从原图计算；“保存”写新快照，不覆盖旧快照；“导出”生成
   自包含原图、模型/actual（若已有）、会话、报告及 SHA manifest。
5. “新安装”按输入的腕侧/安装 ID 新建会话，不改物理状态。关闭窗口只退出 C。

## 在线入口的真实范围

仅在用户明确需要在线检查时运行：

```bash
bash /home/syf/g1_upper_body_ws/scripts/start_camera_c.sh --live
```

复用 A 原 ROS 环境（domain0）只读订阅 `/lowstate`、`/robot_description`、原任务状态；
不是 domain96 下假装接收到 actual。启动不请求相机。点击“连接检查”才连接原 vision_service；
点击“更新 RGB 快照”才复用 B 的客户端取图、下载并作为 preview 保存；停稳采样按钮则按明确选择的 fit/validation 归档。仍需要原服务 model_ready。
实时 actual 现会在显式 RGB 快照时归档 RPC 区间的原始接收窗口（含前后余量），不是正式曝光配对。
模型显示“已接收”也不代表配对合格。
已进行 C 接收器的在线只读检查；尚未验收新版本的现场采图闭环，未启动/重启任何机载服务或控制器。

正式采样始终拒绝：现有 `vision_runtime.py:capture_once` 在请求中直接 `wait_for_frames()`，
尚无证明排除请求前缓冲帧的曝光边界合同；`captured_at_ms` 在推理后生成。
设备时间、请求墙钟、下载完成时间都不能直接替代曝光时刻。也还没有实测的 DDS 反馈传输年龄上界。
需先在原 owner 内设计/审计边界证据，并经用户获准的正常生命周期部署及现场验收，才能开放采样。
当前不是“多等一下”或“只看请求前后角度”可以消除的缺口。

原任务导纳/当前姿态保持继续使用原任务面板；C 没有 ARM、伸指、松手或运动按钮。
现场只在步骤确实需要时确认板固定在所选腕部刚性件，不默认活动手指夹持刚性。

本机旧版 Humble executor 只向订阅回调传 msg，不传 MessageInfo，底层 Python 绑定也没有 GID。
C 在线 actual 已改由专用 C++ 接收器读取；若来源映射尚不可用会保存 null 并拒绝配对。
2026-09-13 的 `lowstate() missing ... info` 崩溃已修复，两种回调的本机分发检查通过；
本次 C++ 接收器及面板修改需要编译；原任务面板/控制器无需重启。

## 离线 actual 与数学核心

`camera_c.py` 提供 `actual_fk`、`static_window`、`pairing`、`solve_transforms`、
`CameraCSession.attach_actual`。已有 actual JSON 可通过以下命令关联到 C 样本：

```bash
/usr/bin/python3 scripts/camera_c.py --session C_SESSION.json --sample-id c_SAMPLE_ID \
  --actual ACTUAL.json --model SAVED_MODEL.urdf --bounds BOUNDS.json --export
```

`BOUNDS.json` 要含本机 `rpc_start_monotonic`、`rpc_end_monotonic`、
`rpc_start_realtime`、`rpc_end_realtime`（秒）。actual 必须提供原逐帧 q、tick、接收时钟、
模型 SHA、唯一来源和 owner/STOP/模式代次，字段以 `static_window` 为准。
缺字段不能填默认“合格”值。即使静态检查通过，本版 `pairing_valid` 仍为 false。
导入不会把保存的 `pairing_valid=true` 当作可信证据，也不能绕过现场边界验收。

在线快照的 C 专用 `TimedVisionClient` 只包装 B 客户端 RPC，不改 B 或另开相机：
RPC finally 中记录 monotonic/realtime，等待 0.15 秒后余量及最多 0.05 秒接收宽限，
冻结窗口后才进入既有下载/检测。保存的区间是本机 Python RPC 调用包络，**不是曝光边界**。
`receipt_window/` 含 `actual.json`、`bounds.json`、`context.json`、原样接收的模型与 SHA manifest。
每条反馈保存 q/dq、tick、接收时间、实际来源 GID、owner 原状态及年龄、模型 SHA、
运动模式和状态变化计数。缺流、无模型、模型变化及无效反馈保留原证据，不补默认值。
RPC/下载失败的采集目录写入失败审计；RPC 已结束时仍尽可能保留窗口。
启动和离线模式均不会自动触发上述采集。

来源发现只是定期 DDS 图观察，不能证明连续唯一性；接收时间也没有传输年龄上界。
这两点与曝光边界缺口一起限制正式同步结论。本版仍不会将静态检查通过升级为配对合格。

数学合同为 `X P_i = G_i Y`，`G_i = inverse(T_pelvis_torso) T_pelvis_wrist`。
OpenCV PARK 接收 `G_i^-1`、`P_i`，输出 X；用 SO(3) 均值估计 Y。不拟合 A TCP，不使用深度选解。
数学函数只是纯计算，不是正式样本准入入口；合成验证只能证明方向/单位，不能证明相机精度。
会话求解只取配对合格且未排除的 fit；现有真实档案因此拒绝求解，没有可导出的实际 X/Y。
旧外参比较/固定 Y 留出残差代码已有，但真实候选及验证链路尚未获得现场数据验收。

## 本轮验证证据

- 导纳增量：诊断包编译成功；5 秒只读 DDS 检查收到 5126 条 actual、9 条 owner、1 条模型，
  actual/owner 的内置发布者句柄映射 GUID 与 ROS 发现列表一致，未映射条数 0，
  本次本地管道最大延迟 2.137 ms。owner 为 TASK_ADMITTANCE，接收器 SIGINT 正常退出。
  该延迟不是 DDS 网络传输年龄上界，也不是图像曝光证明。
- 内联检查确认：停稳确认的导纳窗口可通过静态检查，墙钟跳变只报告；未确认、运动、
  缺 GID 和管道延迟均拒绝，曝光配对门仍关闭。没有新增 test 文件。
- `logs/camera_c_offline_20260913/core_checks.json`：合成 X/Y 恢复最大元素误差约 6.7e-16；
  4 条独立合成留出；退化和六类 actual 区间异常拒绝。合成 actual 不进入真实 C 会话。
- `logs/camera_c_offline_20260913/ui/connected_window.json` 与 PNG：真正 C 后端连接，
  28 行、图像亮像素 126729、离线自动启动/关闭成功。使用 Qt offscreen 渲染，不冒称现场桌面确认。
- 只编译 `g1_tcp_diagnostics`；无新增 test 文件、无全仓测试。
- RViz 插件已编译并注册；本轮先验证独立 Qt 窗口，未验证 RViz 3D/候选轴显示。
  后续增量已通过同包原有 `tcp_a_visual_check` 加载 C 配置进行实际 RViz/Ogre 检查：
  `logs/camera_c_offline_20260913/rviz/` 下有后端/视觉日志及 `rviz_canvas.png`、`rviz_widget.png`。
  画布 784x896，亮像素 2828、彩色像素 1123，检查退出 0，C 后端正常退出。
  Qt 整窗截图未捕获原生 Ogre 子窗口内容时，以单独 Ogre framebuffer PNG 为准。
- `--rviz` 现在订阅专用 `/g1/camera_c/markers`，在 torso_link 内显示原外参 RGB 板投影，
  显式标记 NOT A PAIRED POSE；原安装模型以 SHA 归档，不发布 TF。
  候选/配对腕部预测的 marker 分支也已接线，但没有真实合格候选，不能称现场候选显示已验收。
- `logs/camera_c_offline_20260913/receipt_192bfa1e/checks.json`：mock RPC/下载验证冻结先于下载；
  201 条明确标注 synthetic_actual 的反馈搭配一张历史 RGB 仅检查序列化，导出回放一致且仍拒绝正式配对。
  该数据位于 logs，不是 C 现场数据；没有联网或新增 test 文件。真实 ROS 回调与在线时序仍未验收。

下一步仍需原 owner 曝光边界方案、反馈传输年龄/源证据及获准现场验收。
当前真实归档无合格 C 配对，无法生成或验收实际 X/Y；不能借用 B 图像或合成反馈绕过这一缺口。
