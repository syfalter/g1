# 历史标定参数

本目录收录历史 YAML，用于阅读、模型复算与理解控制链；原始 CSV 采样不进入 Git。

- `right_arm_camera_fit_20260824_185416.yaml`：历史右腕附加载荷拟合，被若干控制入口引用；更换相机、安装件或手部后不能直接沿用。
- `left_arm_right_copy_seed_20260908.yaml`：明确是右侧数值复制，`calibration_status: copied_right_unvalidated`，不是左臂标定结果。
- `right_arm_candidate_poses.yaml`、`right_arm_validation_poses.yaml`、`right_arm_resume_to_pose8.yaml`、`right_elbow_drag_range.yaml`：特定实验的姿态/范围，不是新人的真机动作清单。

需要复核拟合时，应取得对应 CSV、URDF、工具配置和验证姿态，并保留训练与验证的区别。本次整理没有重新标定，没有重算这些参数，也没有更改运行控制器。
