# 来源说明

- `training/serialPort`、`training/include/AutoAimTypes.hpp`、`AppSupport.hpp`、`training/src/AutoAimTypes.cpp`、`AppSupport.cpp`、`training/main.cpp`、标定模板和scaffold测试，来自用户提供的 `RMVtraining-main/RMVtraining-main` 教学起始工程。其中main及接口为本工程适配过的版本；串口结构体保持原样。此目录复用教师提供的基础设施，算法实现位于本工程的视觉核心、跟踪核心和 `training/src/StudentTasks.cpp`。源工程未提供独立LICENSE文件，不将其许可擅自标记为MIT。
- `src/rm_aa_webUI/client/thirdparty/httplib.h` 和 `json.hpp` 为参考工程所附的 cpp-httplib 及 nlohmann/json，保留原文件的版权与许可说明。原教学 Web 服务与静态页面已删除；当前页面及 FastAPI 服务为本项目新增实现。
- `src/rm_aa_vision/models/mlp.onnx` 和 `label.txt` 沿用本地ChenJun/rm_auto_aim参考权重及标签，未经本项目重新训练。保留 `licenses/rm_auto_aim-MIT.txt`。数字ROI预处理约定参照该模型输入。
- 海康相机通过用户提供的MVS SDK接口接入，SDK由用户按厂商条款安装；不把Linux SDK库打包成项目自有代码。

当前适配没有复制参考工程中未使用的BulletModel、KalmanFilter、Planner；使用RM_AA现有算法并扩展交替半径和高度状态。测试与编译缓存位于output，不属于提交源文件。

`src/rm_aa_hardware` 的 ROS2 适配及协议转换代码为本次新增；构建时链接上文 training 中原样保留的 SerialPort.cpp，并包含同一个 Communication.hpp，不另行复制或变更线协议。其参考代码的许可沿用上述来源说明。
