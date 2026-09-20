# 海康工业相机取流

`Source` 的 `camera` 模式现在默认使用海康 MVS SDK，支持 SDK 枚举的 USB3 Vision 与 GigE Vision 相机。普通 UVC 相机仍可通过 `source.camera_backend: uvc` 使用原来的 OpenCV 路径。synthetic、video 和 topic 模式不依赖 SDK。

## SDK 与构建

本地 `Camera_Linux_SDK` 存放的是 Linux 安装包，不能把这个 Windows 目录直接作为 Linux 编译器的链接库目录。Runtime 4.8.1 包提供运行库，但没有开发头文件；本次接口依据完整 `MVS_Linux_STD_V5.1.0_260909_` 包中的 `include/MvCameraControl.h` 和 C++ 示例核对。该完整包内的 x86_64 控制库版本为4.8.2.1。编译与运行应使用同一套完整 MVS 安装。

在运行项目的 Linux 主机上，按 CPU 架构选择完整 MVS 安装包（PC通常x86_64，Jetson通常aarch64），依照包内安装脚本完成安装。x86_64 示例：

```bash
tar -xzf MVS-5.1.0_x86_64_20260909.tar.gz
cd MVS-5.1.0_x86_64_20260909
sudo bash setup.sh
```

默认安装位置为 `/opt/MVS`。先用 MVS 客户端确认能够枚举相机并连续取图，再关闭客户端释放相机独占访问权。USB权限、传输层插件和动态库搜索路径使用官方安装脚本提供的设置；必要时重新打开终端。不要只复制单个 `libMvCameraControl.so`，SDK还依赖其他运行库和传输层组件。

从 RM_AA 根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args \
  -DRM_AA_WITH_HIKROBOT=ON -DMVS_ROOT=/opt/MVS
source install/setup.bash
ros2 launch rm_aa_bringup bringup.launch.py source:=camera device:=0
```

只验证视觉取流时可运行：

```bash
ros2 launch rm_aa_bringup perception.launch.py source:=camera device:=0
```

构建日志应出现 `Hikrobot capture enabled`。Linux 下首次配置时若检测到默认/指定MVS根目录的头文件，自动启用SDK；显式传 `ON` 可避免之前CMake缓存的OFF影响。SDK缺失且选项关闭时仍能构建非海康输入，选择海康输入会清晰报错，不会静默退回UVC。Windows不链接提供的Linux SDK。

## 相机参数

在 `src/rm_aa_bringup/config/perception.yaml` 修改：

```yaml
source:
  camera_backend: hikrobot
  device: "0"
  serial_number: ""
  width: 1280
  height: 720
  fps: 60.0
  exposure_us: 5000.0
  gain_db: 0.0
  grab_timeout_ms: 1000
```

- `device` 为 SDK 枚举序号，**不是 `/dev/video0`**；launch 的 `device` 会覆盖 YAML。
- `serial_number` 非空时优先使用序列号，忽略序号，适合多相机或需要固定设备的场景。
- 使用连续采集、触发关闭、自动曝光关闭、自动增益关闭；曝光和增益由上面参数设置。
- 宽高、曝光、增益和帧率需在具体型号允许范围内。相机节点不支持参数、尺寸步进不合法等情况会报操作名及SDK十六进制错误码，不静默忽略。
- 设置宽高前会将可读取的 OffsetX/Y 设为0；不进行软件缩放。ROI/分辨率改变后，应重新确认标定与此图像匹配。
- `fps` 是相机请求值，实际帧率受曝光、带宽和传输速度限制。
- `grab_timeout_ms` 允许1～5000ms。暂时无帧时重试，节点日志限频提示；关闭节点会在当前有限超时返回后退出。检测到断线或其他错误时停止采集，不自动重连。
- 单色图可由 SDK 转为 BGR8，但没有真实颜色信息，当前红/蓝灯条筛选需要彩色相机。

普通 UVC 设备使用 `camera_backend: uvc`，此时 `device` 可为编号或设备路径；SDK专用曝光、增益参数不用于UVC。

## 数据和资源生命周期

```text
MV_CC_Initialize（进程级）
→ EnumDevices → CreateHandle → OpenDevice
→ 连续模式、参数配置、最新帧策略 → StartGrabbing
→ GetImageBuffer
→ BGR8直接复制 / ConvertPixelTypeEx转换成独立cv::Mat
→ FreeImageBuffer（包括异常路径）
→ 节点最新帧缓存 → 原有检测、分类与PnP
→ 退出：StopGrabbing → CloseDevice → DestroyHandle
→ 进程结束：Finalize
```

图像在交给其他线程之前已拥有独立内存，不会引用已归还SDK的缓冲区。SDK图像队列采用LatestImagesOnly，避免应用处理较慢时持续积压旧帧。

目前时间戳仍为主机取帧/转换完成后的ROS时间，未将相机硬件时间映射到ROS时钟；不能解释成硬件曝光中心时间。真实PnP仍需设置有效标定；未标定时可以取图、检测、分类，但不会输出有效位姿。

## 本次验证范围

在Windows使用实际MVS头文件与官方OpenCV 4.10.0头文件，对 `source.cpp` 的SDK启用/禁用分支完成MSVC编译检查。此检查只生成目标文件，不链接Linux SDK、不连接相机。

新增无硬件Source测试：合成帧内存独立性、非法后端/曝光/超时参数拒绝。实际Linux动态链接、USB/GigE枚举、像素转换和连续取帧仍需在安装MVS的Linux主机及相机上验证；本机无ROS2和相机，不能宣称完成实机联调。
