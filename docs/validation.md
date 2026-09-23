# 验证记录

## 2026-09-20：统一 FastAPI 监控与配置编辑

新增 `src/rm_aa_webUI`，集中管理原生页面、FastAPI/WS 服务、可选 ROS 桥、配置文件编辑和 training 的 C++ HTTP 指标客户端。删除原 bringup 网页、`debug_web.py`、training 的 `web_debug`、旧网页测试及预览脚本；不提供旧 UI 接口兼容。下文早期记录中的 Web 实现已被本次替换。

- FastAPI/Pytest **11/11 通过**：YAML 类型与注释、OpenCV 配置、launch 字面量、语法错误、外部写入冲突、路径范围、预览不落盘、HTTP/WS 多客户端与历史、JPEG 等。
- Windows/MSVC Release 重新构建 `monitor_client` 及原核心成功，原 CTest **20/20 通过**。
- 实际 `monitor_probe.exe` → HTTP → FastAPI → WebSocket 验证通过，两个连续 yaw 样本及 pitch 样本值完整保留。
- 本机 Edge/Playwright 浏览器验证通过：自动曲线、暂停/继续、CSV 下载、折叠、YAML 保存、launch 默认值保存、源码语法拒绝与有效保存、清空、390px 窄屏；无页面 JavaScript 错误。测试只修改 `output/webui-smoke-*` 配置副本。
- 截图：`output/ui-monitor.png`、`ui-config.png`、`ui-mobile.png`；曲线为明确标注的演示数据。

**尚未完成：Linux ROS2 包构建、ROS 话题到网页的现场联调、真实相机和电控联调。** `tools/smoke_ros.py` 已改为验证新 WS/JPEG 接口，但当前 Windows 环境无法执行。参数保存只改变磁盘配置，未实现运行中 ROS 参数热更新。

## 2026-09-20：ROS2 C++ 电控桥接

新增 `src/rm_aa_hardware`，直接复用 training 的 Communication.hpp 和 SerialPort.cpp；没有复制修改协议字段。新增接收姿态、差分关节速度、动态 TF、完整目标消息映射、MPC 轨迹时间取样和收发超时处理。总 launch 增加硬件开关及 mock/sim_time 互斥检查。

控制节点新增可选 roll 反馈，用于相机偏移与 TF 的一致性检查；保留反馈原始整数纳秒用于 TF 查询，避免 double 秒往返转换后超出最新动态 TF 时间。没有将 ROS 控制器改成三轴电机控制。

Windows/MSVC Release 成功构建协议核心，CTest **9/9 项通过**：线协议字节偏移与单位、分包粘包和干扰恢复、坏尾/NaN恢复、无目标/过期标志/模式限制、开火门控、角度跨界差分和断流恢复、pitch/roll 四元数、轨迹时间取样及非法数据拒绝。日志位于 `output/hardware-core-build/Testing/Temporary/LastTest.log`。

新增 Linux ROS2 伪串口测试 `test/test_hardware_pty.py`，覆盖实际节点接收、关节反馈、TF、完整指令、轨迹轴名映射、收发超时与禁火；**当前没有可用 ROS2/Linux 运行环境，该测试尚未执行，ROS 节点尚未完成构建验证**。真实电控、海康及动态 TF 联调也未执行。Python launch/测试脚本完成语法解析，package.xml 和 YAML 完成格式检查。这些检查不替代 ROS2 构建及运行。

## 2026-09-20：RMVtraining 框架、协议和算法适配

对照工作区 `RMVtraining-main/RMVtraining-main` 的任务书、验收表、通信结构体、轨迹预测和外参文档实现 `training/` 入口。保留参考串口及 Web 基础设施，完成四个任务函数，并复用已有视觉、弹道和控制核心。

验证环境：Windows，MSVC 19.50，OpenCV 4.10.0 静态库，Eigen 3.4.0，GoogleTest 1.15.2，Release。根 CMake 已成功构建核心库及测试程序。测试构建禁用海康链接；视频目录读取由单元测试夹具覆盖，不属于运行时图像源。

CTest **20/20 项通过**，包含原有控制测试、新增交替半径/高度跟踪测试、4 项训练入口集成测试以及参考 scaffold 协议/标定校验测试；并行 `-j 4` 运行通过。

| 验证内容 | 结果与边界 |
|---|---|
| 通信布局 | `Communication.hpp` 与参考文件 SHA256 完全一致；所有字段偏移及 64 字节长度静态断言通过 |
| 输出字段 | 校验 m→mm、`crc=0/1/2`、无目标清零、非法数值拒绝、未稳定禁火；Release 保留参考测试的 assert |
| 取图与检测 | 视频读取尺寸/BGR 类型正确；黑图无检测；加载实际 ONNX。未以真实图像集验证分类准确率 |
| PnP | 已知投影生成大小两种板角点，检查解算位置与完整旋转/平移外参；拒绝未标定参数 |
| EKF | 平移与旋转、角度跨界、异常值、超时、两种半径及相邻高度差；运动云台下固定目标坐标和速度保持正确 |
| 预测与弹道 | 弹道核心测试，以及适配入口中重力抬角、距离单位、超限/负预测偏置拒绝 |
| 短时缺测 | 允许未稳定状态的位置预测，禁止开火；过期帧返回无目标 |

构建与 CTest 日志：`output/training-build.log`、`output/training-build/Testing/Temporary/LastTest.log`。本地编译 OpenCV 的源码、依赖安装及构建缓存均位于 `output/`，不属于需要部署的项目源码。

未验证：Linux 上的 `rm_aa_training` 可执行文件构建、真实/伪终端串口收发、Web 浏览器连续访问、海康 SDK 实机取流、ROS2 colcon 与 launch、真实运动目标和实际执行机构闭环。因此这些单元测试不表示已通过任务书的现场验收。相机内外参模板保持未标定，必须填入实测数据后才能启动。

## 2026-09-19：控制编译文本与弹道候选选择修复

本次修复：

- 将 `control_node.cpp`、`test_control.cpp` 中 include 之间的字面量反引号+n 改为真实换行。
- 删除 TF `transform(...)` 调用括号前多余的反斜杠。
- 将弹道候选 `flight_time` 初值从 NaN 改为正无穷，使最短正飞行时间候选能够被选中；无有效候选时仍由有限性检查拒绝。
- 将 ROS 参数返回的 `std::vector<double>` 显式转换成 `Eigen::Vector3d`，用于枪口偏移。
- 扩充现有低弹道测试，检查此目标的俯仰角小于45度，以及飞行时间有限且为正。

验证环境：Windows，MSVC 19.50.35728，Eigen 3.4.0，GoogleTest 1.15.2，Release。

使用 `RM_AA_CONTROL_STANDALONE=ON` 配置，成功构建 `camera_control_core` 和 `test_control`。CTest **14/14 项通过**：几何指向3项、整车跟踪及选板5项、MPC 3项、弹道3项。弹道测试覆盖实际枪口低弹道、横向枪口偏移、非法输入与不可达目标。

构建目录为 `output/control-core-build`，详细测试日志由 CTest 写入 `output/control-core-build/Testing/Temporary/LastTest.log`。

本次没有可用的 ROS2 环境，因此未编译 `control_node`，也未执行完整 `colcon build`、ROS2 launch、话题集成或真实硬件测试。节点源码修正已检查，但不能用独立核心测试代替节点构建验证。
