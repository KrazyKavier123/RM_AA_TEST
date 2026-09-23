# 参数文件位置

项目运行参数 YAML 统一存放在本目录：

| 文件 | 用途 | 总入口参数 |
|---|---|---|
| perception.yaml | 相机、内参、预处理、检测、分类、PnP、调试图 | config |
| control.yaml | 坐标系、反馈话题、弹道、EKF、MPC | control_config |
| hardware.yaml | 串口、命令来源、固定目标 ID、TF 外参、通信超时 | hardware_config |

`bringup.launch.py` 和 `tracking.launch.py` 默认读取本包安装目录中的配置。
单独启动硬件包时必须指定路径（从项目根目录执行）：

```bash
ros2 launch rm_aa_hardware hardware.launch.py \
  hardware_config:="$PWD/src/rm_aa_bringup/config/hardware.yaml"
```

硬件包接收配置路径，不依赖 bringup 包。移动文件不改变 YAML 顶层的节点名。

其他参数来源：

- `src/rm_aa_bringup/launch/`：source/device/video、hardware、web、host/port 等启动默认值。source/device/video 会覆盖视觉 YAML 的对应项。
- `src/rm_aa_hardware/launch/`、`src/rm_aa_webUI/launch/`：独立启动参数。
- 各节点 C++ 的 `declare_parameter`：未被配置覆盖时使用的默认值。
- `src/rm_aa_vision/cmake/Hikrobot.cmake`：MVS_ROOT、RM_AA_WITH_HIKROBOT；`tools/build.sh`：默认构建选项。实际缓存位于 build/，不应作为源码配置维护。
- `tools/requirements.txt`、`src/rm_aa_webUI/requirements*.txt`：Python 依赖版本。
- `src/rm_aa_vision/models/label.txt`：分类标签顺序，必须与模型保持一致。

install/ 中的文件是安装副本或符号链接，output/ 中可能保存工具生成的标定文件；它们不是额外的默认参数源。SDK 安装包内部配置归厂商管理。

修改配置后重启相关节点；复制安装需重新构建，符号链接安装应确认链接指向源码。WebUI 自动扫描 src 下的 config YAML，无需额外更新扫描路径。
