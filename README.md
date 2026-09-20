**RM_AA：C++17 装甲板视觉、跟踪与 RMVtraining 协议适配工程。**

2026-09-20 已按 RMVtraining 要求实现 Linux 独立入口：64字节串口收发、读线程/操作线程、海康取流、检测分类、PnP与实测外参、11维旋转车体EKF、预测指令和数值Web曲线。构建、协议、坐标和配置说明见 [training/README.md](training/README.md)。

原ROS入口保留，视觉像素KF与整车EKF是独立模块。ROS图像网页显示相机观测方位；训练入口数值网页显示PnP、EKF和实际协议输出。两套入口分别启动，不要同时占用同一相机或Web端口。

ROS 电控桥接见 [rm_aa_hardware](src/rm_aa_hardware/README.md)：接收同一 64 字节协议并发布姿态/关节反馈及动态 TF，发送完整目标指令或按时间采样的 MPC 轨迹。通过 `hardware:=true` 启用；默认完整目标消息模式，接入现有 MPC 时需要在硬件 YAML 中选择 trajectory 并配置已知目标 ID。

相机内外参、串口设备、MCU姿态轴向和编号映射需按实物配置。模板标定无效时程序拒绝运行；开火配置默认关闭。编译/单元测试与实机验收范围见 [验证记录](docs/validation.md)。

**目录布局**

ROS入口是 `ros2 launch rm_aa_bringup bringup.launch.py`，构建使用 `colcon build --base-paths src`；参数见 [ROS启动说明](docs/startup.md)。任务书的独立串口入口则使用根目录CMake构建生成的 `rm_aa_training`，详见上面的training说明。

```text
RM_AA/
├── CMakeLists.txt                  独立训练入口构建
├── training/                       双线程主程序、算法适配、原串口协议与Web曲线
├── src/
│   ├── rm_aa_vision/
│   │   ├── include/rm_aa_vision/     C++模块接口、观测数据结构
│   │   ├── src/                     模块实现、ROS节点、离线CLI
│   │   └── models/                  已有MLP权重、标签
│   ├── rm_aa_control/               整车EKF、MPC、弹道与ROS控制节点
│   └── rm_aa_bringup/
│       ├── config/                  YAML参数
│       ├── launch/                  ROS2启动文件
│       ├── scripts/                 ROS→HTTP只读调试桥
│       └── web/                     HTML/CSS/JavaScript
├── tools/                          安装、构建、检查、预览脚本
├── tests/                          Web接口测试
├── docs/                           架构、算法、验证说明
├── licenses/                       第三方许可
└── output/                         检查结果（生成文件）
```

**每个环节的实际实现**

| 环节 | 技术实现 | 源码 |
|---|---|---|
| 相机/视频取流 | 海康MVS SDK；可选OpenCV VideoCapture/UVC；支持视频、合成帧、外部ROS图像话题 | `source.cpp`、`perception_node.cpp` |
| 帧缓存 | 独立采集/处理线程，单槽缓存覆盖旧帧，避免处理积压 | `perception_node.cpp` |
| 图像预处理 | BGR转灰度、亮度二值化、可选形态学开运算（腐蚀再膨胀），默认关闭开运算 | `preprocessor.cpp` |
| 灯条提取 | findContours、minAreaRect、长度/宽度/倾角过滤、轮廓内B/R均值判断颜色 | `detector.cpp` |
| 装甲配对 | 灯条长度比、间距比、高度差、内部灯条过滤；间距/灯长区分大小板 | `detector.cpp` |
| 数字识别 | 透视变换、20×28数字ROI、Otsu二值化、浮点归一化、MLP ONNX、softmax | `classifier.cpp` |
| 分类筛选 | 置信度阈值及negative类别；未通过的几何候选仍以橙框显示 | `classifier.cpp` |
| PnP | OpenCV solvePnPGeneric + IPPE；检查正深度并取重投影误差较小解 | `pose_solver.cpp` |
| 观测角度 | 相机坐标三维观测换算为方位角，只用于显示 | `pose_solver.cpp` |
| 图像跟踪 | OpenCV线性KalmanFilter，状态[u,v,du/dt,dv/dt]，一对一距离关联 | `image_tracker.cpp` |
| 轨迹恢复 | 超时/时间倒退/数值异常处理；新轨迹重新初始化状态与协方差 | `image_tracker.cpp` |
| 框与中间量 | 闭合四边形、外接矩形、类别/大小、图像轨迹编号 | `perception_node.cpp` |
| ROS传输 | PoseArray、CompressedImage及String封装的JSON遥测 | `perception_node.cpp` |
| Web | FastAPI + WebSocket/HTTP；原生HTML/CSS/JS；ROS/training监控与配置文件编辑 | `src/rm_aa_webUI/` |

装甲检测使用传统OpenCV几何方法；神经网络只负责数字分类。模型来自本地rm_auto_aim，名称为`mlp.onnx`，不使用YOLO、TensorRT或OpenVINO。完整来源见 [THIRD_PARTY.md](THIRD_PARTY.md)。

**Ubuntu / ROS2运行**

目标环境：Ubuntu 22.04、ROS2 Humble、OpenCV 4、C++17。需先安装ROS2并初始化rosdep。C++独立核心已在临时Alpine容器中编译，9项测试通过；ROS2节点完整编译、集成及硬件验证尚未完成，详见 [验证记录](docs/validation.md)。

将整个 `RM_AA` 目录复制到Linux文件系统，例如 `~/RM_AA`，然后：

```bash
cd ~/RM_AA
bash tools/install_deps.sh
bash tools/build.sh
source install/setup.bash
ros2 launch rm_aa_bringup perception.launch.py
```

浏览器打开 `http://127.0.0.1:8080`。默认运行C++合成场景，经过实际检测/分类/PnP模块；合成图案不保证与数字模型训练集一致，分类被拒绝时会显示橙框。这不是准确率测试。

海康工业相机（先按 [海康接入说明](docs/hikrobot_camera.md) 安装完整MVS SDK并启用构建）：

```bash
ros2 launch rm_aa_bringup perception.launch.py source:=camera device:=0
```

视频回放：

```bash
ros2 launch rm_aa_bringup perception.launch.py source:=video video:=/absolute/path/video.avi
```

外部工业相机ROS2驱动：

```bash
ros2 launch rm_aa_bringup perception.launch.py source:=topic
```

在配置中设置实际 `source.image_topic` 和 `source.camera_info_topic`。海康可使用上述原生MVS取流，也可通过外部ROS2驱动接入；迈德威视仍需外部驱动。ROS输入需提供有效图像时间戳，时间与节点ROS时钟一致，CameraInfo需与图像同坐标系、分辨率、ROI及畸变模型。普通USB/UVC相机使用camera模式，并在YAML设置 `source.camera_backend: uvc`。

修改配置：编辑 `src/rm_aa_bringup/config/perception.yaml`，或传入 `config:=/absolute/path/custom.yaml`。launch的 `source/device/video` 参数会覆盖YAML中对应项目。配置在启动时读取，运行时设置会明确拒绝，请重启节点生效。

**标定与大小板尺寸**

真实相机和视频默认 `calibration.calibrated: false`，此时仍输出检测与图像轨迹，但不输出PnP。填入真实内参、畸变参数和标定分辨率后改为true。参数与当前图像尺寸不一致时不解算PnP。合成场景使用明确标识的虚拟内参。

默认小板135×55mm、大板225×55mm，来自本地rm_auto_aim。按题目实体装甲尺寸修改 `dimensions.*`，这里的四点对应两条灯条的端点，不能直接替换成装甲外壳宽高。尺寸单位为米。

`detector.own_color: red`对应观测蓝色，`blue`对应观测红色。配置不会从裁判系统或串口自动更新。

可以用棋盘格标定脚本生成完整参数文件（下例为9×6内角点、20mm方格）：

```bash
python3 tools/calibrate_camera.py --images 'calibration/*.png' \
  --cols 9 --rows 6 --square-m 0.02 --output output/calibrated.yaml
ros2 launch rm_aa_bringup perception.launch.py source:=camera config:="$PWD/output/calibrated.yaml"
```

脚本要求至少6张同分辨率可用视图，打印整体和逐图重投影误差。实际采集应覆盖不同位置与倾角；该脚本尚未用你的相机数据验收。

**话题与Web**

| 输出 | 类型 | 内容 |
|---|---|---|
| `/observations` | geometry_msgs/PoseArray | 本帧有效的相机坐标系PnP观测 |
| `/telemetry` | std_msgs/String | schema=1 JSON，含逐候选分类/位姿、图像轨迹、性能 |
| `/debug/image/compressed` | sensor_msgs/CompressedImage | 分类通过绿色、待确认橙色、轨迹中心青色 |
| `/debug/mask/compressed` | sensor_msgs/CompressedImage | 二值化中间图 |

遥测中的 `pose_array_index` 将候选与PoseArray索引关联；PnP无效的候选`pose`为null。默认所有话题采用best-effort、深度1。Web按独立默认10Hz频率获取JPEG，实际处理循环可更快。`processing_ms`不含JPEG编码和Web耗时，另有`debug_ms`与`frame_age_ms`。

Web默认只监听本机；需要从同一局域网查看时启动 `host:=0.0.0.0`，再访问机器人IP的8080端口。统一页面提供指标曲线、ROS图像及源码配置编辑；指标走 `/ws`，图像走 `/api/frames/image` 和 `/api/frames/mask`，服务状态为 `/api/status`。过期来源和指标在页面标记，详细说明见 [WebUI](src/rm_aa_webUI/README.md)。

**测试和单模块运行**

```bash
bash tools/test.sh
./build/rm_aa_vision/test_perception --gtest_filter='Detection.*'
./build/rm_aa_vision/test_perception --gtest_filter='Pose.*'
./build/rm_aa_vision/test_perception --gtest_filter='Classifier.*'
./build/rm_aa_vision/test_perception --gtest_filter='ImageTracking.*'
ros2 run rm_aa_vision perception_cli \
  src/rm_aa_vision/models/mlp.onnx src/rm_aa_vision/models/label.txt /tmp/plates.png
```

CLI增加最后一个图像路径参数即可处理单张图片。它测试实际C++预处理、检测和分类，不需要相机或ROS消息输入。

也可以在仅有CMake/OpenCV/nlohmann-json/GTest的Linux环境独立编译核心，不安装ROS：

```bash
cmake -S tools/standalone -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core -j2
ctest --test-dir build/core --output-on-failure
```

已安装Docker且可下载官方ROS镜像时，可进行容器内全量检查：

```bash
docker build -t rm-aa-verify:local -f tools/Dockerfile .
docker run --rm rm-aa-verify:local
```

镜像构建执行C++及Web测试；最后一条命令执行合成图像经过C++、ROS消息和HTTP的集成检查。

Windows只预览网页和检查资源：

```powershell
cd D:\RM_Auto_Aiming\RM_AA
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
.\.venv\Scripts\python.exe tools/check_workspace.py
.\.venv\Scripts\python.exe -m pip install -r src/rm_aa_webUI/requirements-test.txt
.\.venv\Scripts\python.exe -m pytest src/rm_aa_webUI/tests/test_server.py -q
powershell -File src/rm_aa_webUI/run.ps1 --demo
```

`--demo` 只生成明确标注的 `demo.` 曲线，不运行 C++ 链路。正常监控请去掉该参数；ROS 总入口的 `web:=true` 会启动新服务，training 则向独立启动的服务发送指标。参数页保存源码 YAML/launch，重启相关节点后生效，不能视为运行中热更新。详见 [WebUI 启动与接口](src/rm_aa_webUI/README.md)。

详细数据流、限制和验证记录分别见 [架构说明](docs/architecture.md)、[验证记录](docs/validation.md)。
