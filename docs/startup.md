# 统一启动入口

`rm_aa_bringup/launch/bringup.launch.py` 复用 `tracking.launch.py`，一次启动项目已有的视觉、控制和 Web 调试节点。相机专有驱动、实际云台驱动及定位/TF 数据源需要外部提供。

```text
bringup.launch.py
└── tracking.launch.py
    ├── perception.launch.py
    │   ├── perception_node
    │   └── rm_aa_webui/server.py（web=true，FastAPI + WebSocket/HTTP）
    ├── control.launch.py
    │   └── control_node
```

## 构建与启动

在已配置 ROS2 Humble 的 Linux 环境、RM_AA 根目录执行：

```bash
bash tools/build.sh
source install/setup.bash
ros2 launch rm_aa_bringup bringup.launch.py
```

新 Web 服务需先安装 `src/rm_aa_webUI/requirements.txt` 中的 Python 依赖，具体 Python/ROS 环境与单独启动方式见 [WebUI说明](../src/rm_aa_webUI/README.md)。网页可编辑源码配置；保存后需重启相关节点，复制安装的配置还需重新构建。

默认 `source:=video`，需要指定视频目录或文件，开启控制和 Web。没有外部 TF/关节反馈时控制节点会等待有效输入。浏览器地址为 `http://127.0.0.1:8080`。

真实海康相机（先按 [海康接入说明](hikrobot_camera.md) 安装并编译MVS支持；需要真实标定、外部云台反馈及 TF；普通UVC相机另设 `source.camera_backend: uvc`）：

```bash
ros2 launch rm_aa_bringup bringup.launch.py source:=camera device:=0
```

外部 ROS 相机驱动输入：

```bash
ros2 launch rm_aa_bringup bringup.launch.py source:=topic
```

自定义配置：

```bash
ros2 launch rm_aa_bringup bringup.launch.py \
  source:=camera device:=0 \
  config:=/absolute/path/perception.yaml \
  control_config:=/absolute/path/control.yaml
```

## 启动参数

| 参数 | 默认值 | 用途 |
|---|---|---|
| `config` | bringup安装目录/config/perception.yaml | 视觉配置 |
| `control_config` | bringup安装目录/config/control.yaml | 控制配置 |
| `source` | video | camera/video/topic |
| `device` | 0 | 相机设备 |
| `video` | 空 | 视频路径；配合source:=video |
| `web` | true | 是否启动Web |
| `host` / `port` | 127.0.0.1 / 8080 | Web监听地址 |
| `use_sim_time` | false | 沿用control.launch行为，仅显式配置控制节点；其他数据源需使用一致时钟 |

`source/device/video` 会覆盖视觉 YAML 中对应值。可通过 `--show-args` 查看启动参数。现有 CMake 安装整个 launch 目录，新入口无需单独增加安装项。

2026-09-19 已修复先前发现的 include 拼接、TF 调用转义和弹道候选时间 NaN 问题，并修正枪口偏移参数到 Eigen 向量的转换。独立控制核心已构建成功，14/14 项测试通过；这不代表已完成 ROS2 节点构建或集成验证，完整运行仍需 ROS2 构建成功及所需输入就绪。见 [验证记录](validation.md)。
