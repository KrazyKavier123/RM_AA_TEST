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