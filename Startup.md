# 安装依赖
```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev

# 用海康相机时再装 SDK（本机已装在 /opt/MVS，安装包在 RM_AA 里）
sudo dpkg -i ~/Desktop/RM_AA/SDK/MVS-5.1.0_x86_64_20260909.deb
```

# 串口权限

```bash
sudo usermod -aG dialout $USER    # 之后重新登录生效
```

# 改参数

参数都在 student/src/StudentTasks.cpp 顶部，改完必须重新 cmake --build

```CPP
const std::string kMode = "video";      // "video" 视频回放 / "camera" 真实相机
const std::string kBackend = "hikrobot";// camera 模式："hikrobot" 或 "uvc"
const std::string kDevice = "0";        // uvc：设备号或 /dev/videoX；hikrobot：枚举序号
const std::string kSerial = "";         // 填 "DA0823281" 之类可固定相机
const std::string kVideoPath = "../videos/video";
const int kWidth = 1440, kHeight = 1080;   // 必须与 camera.yaml 的标定分辨率一致
const double kExposureUs = 20000.0, kGainDb = 16.0;

// MPC：把瞄准角规划成满足角加速度上限的指令（预测 kMpcSteps*kMpcDt = 0.5 s）
const int kMpcSteps = 25, kMpcIters = 300;
const double kMpcDt = 0.02;
const double kMpcAngleW = 40.0, kMpcRateW = 1.0, kMpcEffortW = 0.1;
const double kMpcYawAcc = 6.0, kMpcPitchAcc = 4.0;   // 角加速度上限 (rad/s^2)
const double kMpcTol = 1e-5;
```

相机参数：
```bash
cd ~/Desktop/RMVtraining
cp config/camera.yaml.example config/camera.yaml
```
填入实测内参、畸变、相机→云台外参，并把 calibrated 改成 1


| 项目 | 当前情况 | 需要处理 |
|---|---|---|
| 标定误差 | `mean_reprojection_error_px: -1.` | 填入真实标定误差；负数会被校验拒绝 |
| 配置文件路径 | 只有 `camera.yaml.example`，没有 `camera.yaml` | 补齐后另存为 `config/camera.yaml`，或启动时直接指定现有文件 |
| 取流模式 | `kMode = "video"` | 接真实相机需改为 `"camera"` 后重新编译 |
| 视频来源 | 默认路径 `../videos/video`，从仓库根目录运行时对应位置不存在 | 如果用视频，需要修改路径 |
| 控制器串口 | 当前未发现 `/dev/ttyACM*` 或 `/dev/ttyUSB*` | 接入控制器，确认实际设备名、权限和波特率 |
| 控制器数据 | 程序依赖合法串口输入才开始取流 | MCU须持续发送64字节协议帧，`status` 为0或5 |
| 标定分辨率 | 配置和采集设置都是1440×1080 | 确认这些内参确实在该分辨率下标定 |





# 编译
```bash
cmake -S . -B build-rmvtraining -DCMAKE_BUILD_TYPE=Release
cmake --build build-rmvtraining -j
```

一些情况下可能需要指定CV。

```bash
cmake -S . -B build-rmvtraining -DCMAKE_BUILD_TYPE=Release \
      -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

# 启动

```bash
cd ~/Desktop/RMVtraining     # 必须是仓库根：models/ 在这里
./build-rmvtraining/rmv_training /dev/ttyACM0 115200 config/camera.yaml 8080
```
<串口设备> <波特率> <相机标定文件> [Web端口]，Web 端口省略时默认 8080。

# 摄像头 / 视频画面

从仓库根目录运行，无需串口即可预览真实相机或已有视频：

```bash
./build-rmvtraining/rmv_training --view config/camera.yaml blue
```

最后一项是要检测的装甲板颜色 `red` / `blue`，默认 `blue`。
输入来源仍由 `student/src/StudentTasks.cpp` 顶部的 `kMode` 决定：
`camera` 使用 `kBackend` / `kDevice`，`video` 使用 `kVideoPath`。
视频路径和 `models/` 路径相对于运行目录，标定文件由命令行指定。
预览仅运行取流、识别和 PnP，不需要电控数据。
原串口启动方式也会显示画面，仍需持续收到有效控制器数据。

显示参数位于 `vision/include/View.hpp` 的 `ViewConfig`：
`enabled` 开关、`scale` 初始窗口比例、`axis` 方向轴长度（米）、
`thickness` 线宽、`font` 字号。修改后重新编译。
无桌面环境时将 `enabled` 设为 `false`。

画面四角框表示检测到的装甲板，绿框为接受的数字目标，橙框表示未接受。
文字显示模型类别与置信度，`[ignored]` 表示低于分类阈值或不是 1～5 号。
PnP 成功时绘制 X 红、Y 绿、Z 蓝三轴，蓝轴为装甲板物体系 +Z 法线；
PnP 未通过重投影误差筛选时只显示检测框。
这里的框是装甲板检测框，不是相机标定棋盘格。
采集/视频分辨率须与标定一致，显示缩放不改变算法输入。
按 Q / Esc 或关闭窗口退出。取流失败不再补黑帧，不生成虚拟输入。
