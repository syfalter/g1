# 实验数据另行交接

本目录在开发机约 108 GB，不随源码 Git 上传。这里只保留说明，原有本地数据没有被删除。

| 本地子目录 | 主要用途 |
| --- | --- |
| `task_panel/` | 录制库、准备位、面板选择与编辑设置 |
| `task_sessions/` | 会话状态、计划、采集上下文、执行证据 |
| `offline_trajectory_library/`、`offline_history/` | 不可变归档、历史场景和离线复算 |
| `rgbd_history/`、`vision_captures/` | 彩图、深度、内参、时间戳和目标 |
| `tcp_a/`、`camera_b/`、`camera_c/` | 标定采样与分析 |
| `rail_deployment/`、`action_releases/` | 轨迹候选、发布数据、运行时快照与激活凭据 |
| `*_cache/`、`*_previews/` | 派生缓存、预览和对比 |

`scripts/run_onboarding_checks.sh` 使用合成输入，不依赖这些数据。

需要回放具体任务时，请向维护者领取**一个明确版本的最小工作副本**，包含轨迹/计划、相关模型资源、配置、来源和 SHA256 清单。不要只拿一份轨迹 JSON 却漏掉相机采集状态、手型、模型或配套业务库。

归档中的 `archive_manifest.json`、`context/robot.urdf` 和模型快照应一起保留；可参考 `scripts/task_plan_archive.py --verify` 的完整性检查。验证哈希不等于允许执行。

面板选择 sidecar 与原库的文件身份有关，迁移时优先使用面板导出，并核对导入后的当前选择。编辑工作副本与唯一原始记录分离。发布包和线上 API/native 实际加载的哈希必须另行核验，不能用面板选择代替。

对外共享前检查图像、机载凭据、网络地址和实验对象信息；不要直接取消 `.gitignore` 后上传整个目录。
