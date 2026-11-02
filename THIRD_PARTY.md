# 第三方来源与本地快照

## Unitree

SDK2、本体描述和 ROS 2 消息的来源及 commit 见 [dependencies.repos](dependencies.repos)。厂商完整仓库、预编译库和 meshes 由相应依赖获取，本项目不宣称重新实现了厂商底层控制。

[integrations/unitree_sdk2/](integrations/README.md) 只保存本地新增程序和 CMake 差异，同时保留上游 SDK2 许可证副本 `UPSTREAM_LICENSE`。这不意味着本仓库所有其他文件自动使用该许可证。

## RH56DFX 模型

来源为 `ookkshirsagar/rh56dfx_description`，本地 provenance 记录 commit `77491738aaed410d1b668749160e7b943f6fdd46`。原 README、MIT 许可证及模型保留在 [assets/rh56dfx_description](assets/rh56dfx_description/README.md)。

这是社区模型，不是厂家认证的安装/碰撞模型。上游特别指出 meshes 来源于厂家 CAD，仍受厂家的使用条款约束；本仓库没有重新授予模型权利。由模型导出的工作台手部几何保留各自 provenance。

## 灵巧手与视觉服务

来源为 `Flowhist/g1-rh56dfx-control`，上游基线 commit `304a1ce989c5db8d282c128a74f7309c84b55f8e`。本地开发目录中有四个修改文件，本仓库保存当前源码内容：

- `recognize/pedal_center_package_v1/vision_runtime.py`
- `recognize/pedal_center_package_v1/vision_worker.py`
- `src/vision/vision_api.hpp`
- `src/vision_service.cpp`

这些文件作为普通源码快照收录；没有改动上游远程、没有创建带未提交差异的 submodule，也没有将嵌套 `.git` 上传。模型权重 `.pt` 与厂家 PDF 手册不在本次提交中，按上游说明另行取得。

该快照未附顶层 LICENSE，本次不推断或扩大它的再分发授权。对外再分发、商用或进一步开放之前，应向对应作者/权利方确认适用范围。原 [AGENTS.md](third_party/g1-rh56dfx-control/AGENTS.md) 的硬件使用约束保留。

## 本地历史资料

工作台几何、实验配置与报告的来源说明保留在原文件及 provenance 中。顶层没有新增一份覆盖所有资料的开源许可证；内部交接、公开可读与自由再授权不是同一概念。
