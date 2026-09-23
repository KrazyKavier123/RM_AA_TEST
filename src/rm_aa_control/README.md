# 云台相机跟踪控制框架

C++17 / ROS2（目标环境 Humble），订阅现有 `rm_aa_vision` 的 `PoseArray`，实现小车状态 EKF、最近目标板选择和双轴姿态 MPC。算法库 `camera_control_core` 不依赖 ROS，可单独编译测试。

参考了 `参考代码/SuperPower/26UL_sp_vision_25_nonros-main/tasks/auto_aim/target.cpp` 的车体中心与板面几何建模、`tracker.cpp` 的跟踪组织，以及 `planner/planner.cpp` 的分轴预测控制思路；本包重新实现面向相机拍摄的模型和求解器，没有复制参考项目源码或引入 TinyMPC。

## 数据链路

```text
rm_aa_vision /observations (相机坐标，图像采集时间)
   → 按观测时间查询 TF → 固定系目标板位置与板面法向
   → 小车 EKF → 最近板 + 切换滞回 → 未来板位置 / 视线姿态
   → yaw、pitch MPC ← /gimbal/joint_states (实测角度和角速度)
   → /gimbal/trajectory → 用户的云台驱动适配器
```

| 话题 | 类型 | 约定 |
|---|---|---|
| `/observations` | `geometry_msgs/PoseArray` | 直接兼容视觉输出；SensorDataQoS；可通过 `observations_topic` 改名 |
| `/gimbal/joint_states` | `sensor_msgs/JointState` | 必须提供时间戳、两个命名关节的 position 和 velocity；单位 rad、rad/s |
| `/gimbal/trajectory` | `trajectory_msgs/JointTrajectory` | 每周期重规划，k=1…N 的角度、角速度、角加速度；yaw 为连续角 |
| `/target/state` | `nav_msgs/Odometry` | 中心位置、速度、板0相位和相位角速度；pose/twist 均在固定系，因此 child_frame_id 也为固定系；不广播车体 TF |
| `/target/selected_plate` | `geometry_msgs/PointStamped` | 所选板当前预测位置，固定系 |
| `/control/status` | `std_msgs/String` | 等待目标/反馈、TF或规划不可用、未收敛、跟踪板索引和估计半径 |

输出话题为相对名，以上是默认无 namespace 时的全名。状态/选板诊断随成功规划一起发布。板索引只在一次跟踪期间稳定，重捕获后重新编号。

## 模型与边界

EKF 状态为 `[cx,vx,cy,vy,cz,vz,theta,omega,r]`。采用匀速平移与匀角速度预测、白噪声加速度过程模型。第 i 块板的测量模型为：

```text
alpha_i = theta + 2*pi*i/plate_count
h_i(x) = [cx-r*cos(alpha_i), cy-r*sin(alpha_i), cz, alpha_i]
```

使用解析 Jacobian、角度残差归一化、位置/角度联合关联门限、NIS 异常值拒绝、Joseph 协方差更新。每帧各板与各观测最多关联一次。丢失后短时预测，超过 `tracker.timeout_s` 重新捕获；过期、未来、零时间戳和乱序观测被丢弃，ROS 时间倒退时重置。

当前假设**单辆小车、各板等间隔、同高度、同半径**。默认4块板，可设置1～16块。半径有初值和允许范围，并参与估计。单块板且姿态变化不足时，中心与半径不可充分观测，初值会影响中心估计。`theta` 是板0内法向的相位；安装偏角未标定时不能等同于小车行进航向。

现有视觉 `PoseArray` **没有小车ID、板ID、分类置信度**，且视觉会对未通过分类的几何候选求解 PnP。本模块以所有有效位姿作为候选，初始化选最近观测，后续按几何门限关联。多辆近邻小车或误检可能发生串轨；需要多车时应先扩展带身份和质量字段的视觉消息。这里没有用降频的 JSON telemetry 代替实时控制输入。

“就近”定义为**预测板中心到相机光心的三维欧氏距离**。候选包括模型生成的全部板；设置 `switch_margin_m` 抑制距离接近时来回切换，设0可用严格最近策略。MPC 一次预测窗内固定板索引，下一个周期重新选板。

每轴 MPC 状态 `[angle,rate]`，输入角加速度，使用精确离散双积分模型：

```text
A = [[1,dt],[0,1]], B = [dt²/2,dt]
min Σ (x[k]-xref[k])'Q(x[k]-xref[k]) + R*u[k]²
subject to |u[k]| <= max_acceleration
```

采用凝聚后的凸二次规划和投影梯度法，检查投影梯度残差；未收敛不发布新轨迹。起点来自实测反馈，按反馈年龄做短时匀速外推；参考由未来车体状态计算，yaw 连续展开跨越 ±π。**当前只对角加速度施加硬约束**，角速度通过代价约束，机械角度/角速度限位需在驱动适配器落实，或进一步扩展 QP 状态约束。

## 坐标和硬件接入

固定系默认 `odom`，重力沿 -Z。云台基座 `gimbal_base` 必须是**不随 yaw/pitch 关节转动的基座系**，X前、Y左、Z上。输入输出约定 yaw 向左为正，pitch **向上为正**；标准绕 +Y 的关节转动通常向下，适配器要统一符号、零位及连续角反馈。

视觉 optical 系为 X右、Y下、Z前。PnP 的目标板局部 +Z 默认作为指向小车中心的内法向；若实际安装/角点约定相反，设置 `plate_normal_sign: -1.0`。板法向接近竖直时无法估计平面相位，会被拒绝。

必须由编码器/robot_state_publisher/定位模块提供：

```text
odom → gimbal_base → [yaw/pitch 链及标定外参] → camera_optical_frame
```

观测转换使用**图像采集时刻**的动态 TF；规划使用最新有效反馈时刻的 TF，并在预测窗内视基座不动。相机视线计算支持下述光心偏移补偿，移动基座需要进一步扩展基座预测。不能在真实转动云台上用静态 optical 变换替代编码器 TF。

无目标、反馈超时、TF 缺失或 MPC 不收敛时不发布新轨迹。驱动适配器应根据消息接收年龄实行独立超时保持/停止，避免继续执行缓存的旧预测窗；不要将“没有新消息”解释为永远保持上一条运动指令。本包不包含特定型号电机或串口驱动。

## 相机光心偏移标定

`camera_to_pitch_optical_m` 表示相机光心指向 P（俯仰）轴中心的向量，使用相机 optical 坐标轴（X右、Y下、Z前），单位为米。当前 YAML 暂留零偏移，等待确认测量值 `[67.11, 0, 51.5]` 的单位与轴向。修改后需重启节点。

若测量值是 optical 坐标系下的毫米，填写 `[0.06711, 0.0, 0.0515]`；若使用零位云台的 X前、Y左、Z上且单位为毫米，则填写 `[0.0, -0.0515, 0.06711]`。

视线求解先取反并转换坐标，得到俯仰架坐标系下 P 轴到光心的偏移 `offset`，再逐个预测时刻求解：

```text
target = Rz(yaw) * Ry(-pitch) * (offset + distance * [1,0,0])
```

模型要求 yaw/pitch 两轴相交于基座原点，零位光轴朝 +X，相机没有额外安装旋转。无有效前向解时报告 `planning_unavailable`。

此参数只用于规划，不发布或修改 TF。外部相机 TF 必须使用同一标定，观测仍按采集时间通过 TF 转换。如果 TF 父系为俯仰架（前/左/上），相机 optical 子系平移应为 `[-vector_z, vector_x, vector_y]`，其中 vector 是上述 optical 坐标系下以米为单位的参数。非零标定必须与外部 TF 保持一致。

## 构建和启动

在已安装 ROS2 Humble 的 Linux 环境执行（工作目录 `RM_AA`）：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to rm_aa_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select rm_aa_control --event-handlers console_direct+
colcon test-result --verbose

# 视觉已运行时，只启动控制节点；先在 src/rm_aa_bringup/config/control.yaml 中设置实物参数和话题
ros2 launch rm_aa_bringup control.launch.py

# 同时启动现有视觉和控制（source/config/device/video/web 参数沿用视觉 launch）
ros2 launch rm_aa_bringup tracking.launch.py source:=topic

核心算法可脱离 ROS2 测试（需要 Eigen3、GoogleTest、CMake、C++17 编译器）：

```bash
cmake -S src/rm_aa_control -B build/control-core -DRM_AA_CONTROL_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/control-core --config Release
ctest --test-dir build/control-core -C Release --output-on-failure
```

测试覆盖运动和转动的小车估计、可见板切换、异常/乱序/丢帧、最近板与滞回、角度跨界、MPC 动力学与加速度界限、反馈闭环收敛、求解失败。

2026-09-14 本地验证：Windows MSVC 19.50、Eigen 3.4.0、GoogleTest 1.15.2，Release 核心编译成功，8/8 C++ 测试通过（日志 `RM_AA/output/control_ctest.log`）。本机当时没有可用的 ROS2 安装，因此尚未执行 ROS2 节点构建和实物联调；以上结果不代表这些检查已通过。
