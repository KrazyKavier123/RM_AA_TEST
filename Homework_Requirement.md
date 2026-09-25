# RMVtraining 视觉组大作业

本仓库是新赛季视觉组个人大作业的起始工程。题目只覆盖装甲板自瞄。工程提供串口收发、Web Debug 和双线程 `main` 基本框架；取流、识别、位姿解算与 EKF 预测由学生完成。

## 作业边界

教师提供：

- 保持 64 字节布局不变的通信结构体及串口读写接口；
- 读线程和操作线程的组织框架；
- 数值曲线 Web Debug 接口；
- 可通过编译的最小 `CMakeLists.txt`。

学生完成：

- 开发环境、第三方依赖和相机 SDK 配置；
- 相机初始化、取流、像素格式转换及异常处理；
- 相机内参和相机到云台的外参标定，并填写配置；
- 装甲板检测、数字分类、PnP 位姿解算；
- 面向旋转车体的 EKF 跟踪与预测；
- 算法结果到原通信结构体的正确映射。

详见 [大作业任务书](docs/大作业任务书.md) 和 [验收与评分](docs/验收与评分.md)。

## 构建

仓库不提供依赖安装脚本、容器或完整环境配置。请先按照 [环境依赖](环境依赖.md) 自行安装依赖，并根据所选相机 SDK 和识别方案补充自己的构建配置。

最小构建流程：

```bash
cmake -S . -B build-rmvtraining
cmake --build build-rmvtraining -j
```

构建成功只说明起始框架可编译，不代表相机、识别模型或标定参数已经配置完成。

## 运行

完成标定后，将模板复制为自己的配置文件并填入实测数据。程序参数依次为串口设备、波特率、相机配置和可选的 Web 端口：

```bash
./build-rmvtraining/rmv_training /dev/ttyACM0 115200 config/camera.yaml 8080
```

Web Debug 默认监听本机地址，可在 `http://127.0.0.1:8080` 查看。`student/src/StudentTasks.cpp` 中的四个函数初始为安全空实现；未完成取流和算法前，程序不会产生有效目标或允许开火。

## 开始前检查

1. 阅读 [通信结构体](通信结构体.md)，不得调整字段、顺序、类型或帧长。
2. 复制 `config/camera.yaml.example` 为自己的相机配置文件，完成标定后替换全部占位值。
3. 选择传统视觉或神经网络路线，并确保最终输出统一的装甲板角点、大小、编号和置信度。
4. 使用 Web Debug 观察耗时、PnP 和 EKF 数据，但不要在处理线程中加入阻塞式调试代码。

## 提交要求

- 完整源代码与可复现的构建说明；
- 填写后的相机标定文件、标定图片和误差说明；
- 神经网络路线所需的离线模型与来源说明；
- 实验报告，说明坐标系、算法设计、关键参数、测试结果和已知问题；
- 不提交构建目录、IDE 缓存或与运行无关的大文件。

## 已移植的算法链路

本工程已把 `../src` 的算法架构移植进初始框架（纯 C++，无 ROS）。四个任务函数在
`student/src/StudentTasks.cpp` 中把移植模块串成完整链路：

| 任务 | 模块（来源） | 作用 |
| --- | --- | --- |
| `get_pic` | `vision/src/Source.cpp`（← `src/rm_aa_vision/src/source.cpp`） | 真实相机（V4L2）或视频取流，按视频帧率回放 |
| `armor_detect` | `vision/src/Preprocessor.cpp`、`Detector.cpp`、`Classifier.cpp` | 灰度二值化 → 灯条提取与配对 → 数字分类 |
| `armor_solve` | `vision/src/PoseSolver.cpp`（← `src/rm_aa_vision/src/pose_solver.cpp`） | 大小装甲板 IPPE PnP + 相机到云台外参 |
| `ekf_predict` | `tracking/src/Tracker.cpp`、`tracking/include/Ballistic.hpp`、`tracking/src/Mpc.cpp`（← `src/rm_aa_control`） | 11 维旋转车体 EKF、选板、弹道指向与双轴 MPC |

算法相关参数集中在 `student/src/StudentTasks.cpp` 顶部（取流、检测/分类阈值、PnP 尺寸与误差上限、跟踪半径与超时、弹速/重力/枪口偏移、MPC 步长与角加速度上限）；相机内参、畸变与相机到云台外参
仍由 `config/camera.yaml` 提供。数字分类模型与标签在 `models/`（来自 rm_auto_aim，MIT）。

`armor/` 是题目提供的参考实现（`BulletModel`、`KalmanFilter`、`Planner`），已被上述模块取代，
不再参与构建，可按需删除。

取流支持两条链路，参数集中在 `student/src/StudentTasks.cpp` 顶部：

- **视频回放**：`kMode = "video"`，`kVideoPath` 指向视频文件或目录（相对运行目录，默认
  `../videos/video`），按视频自身帧率回放。
- **真实相机**：`kMode = "camera"`，`kBackend` 选 `"hikrobot"`（MVS SDK）或 `"uvc"`（OpenCV/V4L2）。
  海康相机用 `kDevice` 选枚举序号，或用 `kSerial` 按序列号固定设备；`kExposureUs`、`kGainDb`
  控制曝光与增益，`kWidth`/`kHeight` 需与 `camera.yaml` 的标定分辨率一致。

海康 MVS SDK 取流由 CMake 选项 `ENABLE_HIKROBOT` 控制：装有 SDK（`/opt/MVS` 存在）时默认开启，
可用 `cmake -S . -B build-rmvtraining -DENABLE_HIKROBOT=OFF` 关闭，此时只保留 UVC/V4L2 与视频。
海康相机的打开、参数设置、像素格式转换与异常处理在 `vision/src/Source.cpp` 中，另有 `uvc`
和视频两条分支。
