**RM_AA：ROS2 / C++17 视觉观测与调试工作空间。**

当前实现取流、图像预处理、全部候选装甲闭合框、数字识别、大小板区分、PnP、二维图像轨迹、ROS消息及Web观测。代码按模块拆分，可以独立测试。

**当前交付不构成请求中的完整自瞄系统：没有实现自主击打目标选择、弹道拦截、击打时刻装甲预测、执行机构控制或串口瞄准指令。跟踪模块是图像坐标中的线性KF，不是整车EKF。页面中的yaw/pitch是相机观测方位角，不是云台目标值。**

本工作空间定位为不连接执行机构的视觉观测工程。请按下表评估实际交付范围，不能以网页能够显示图像判断完整自瞄已完成。

**目录布局**

整个项目的统一启动入口是 `ros2 launch rm_aa_bringup bringup.launch.py`，启动视觉、控制和可选 Web；无硬件接口演示加 `mock_gimbal:=true`。构建步骤、输入要求和全部参数见 [统一启动说明](docs/startup.md)。该入口复用现有模块，当前控制源码问题仍需解决后才能完成全链路运行。

```text
RM_AA/
├── src/
│   ├── rm_aa_vision/
│   │   ├── include/rm_aa_vision/     C++模块接口、观测数据结构
│   │   ├── src/                     模块实现、ROS节点、离线CLI
│   │   ├── models/                  已有MLP权重、标签
│   │   └── test/                    原生C++测试
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
| 相机/视频取流 | OpenCV VideoCapture；Linux UVC走V4L2；支持视频、合成帧、外部ROS图像话题 | `source.cpp`、`perception_node.cpp` |
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
| Web | rclpy订阅 + Python标准库HTTP服务；原生HTML/CSS/JS轮询；不用CDN | `debug_web.py`、`web/` |

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

Web默认只监听本机；需要从同一局域网查看时启动 `host:=0.0.0.0`，再访问机器人IP的8080端口。页面只读，所有数据来自观测节点。HTTP入口为 `/api/status`、`/frame.jpg`、`/mask.jpg`。超过2秒未收到遥测会标记数据过期。

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
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
powershell -File tools/preview_web.ps1
```

`--demo` / `preview_web`只是Web界面夹具，不运行C++链路，页面会明确标注，帧率/PnP等真实指标留空。它与ROS启动后的C++合成场景不同。

详细数据流、限制和验证记录分别见 [架构说明](docs/architecture.md)、[验证记录](docs/validation.md)。
