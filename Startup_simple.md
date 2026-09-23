# 使用虚拟环境，同时保留系统 ROS Python 包
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r src/rm_aa_webUI/requirements.txt

- 进入项目后激活环境

```
source .venv/bin/activate
```

- 确认当前解释器

```
which python
python --version
```

- 退出环境

```
deactivate
```

# 检查缺少的库依赖

```
sudo rosdep init
rosdep update
```

检查当前项目缺少哪些依赖：
```
rosdep check \
  --from-paths src \
  --ignore-src \
  --rosdistro humble
  ```

安装缺少的依赖：
```
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

参数含义：
- --from-paths src：扫描 src/ 下所有 ROS 包。
- --ignore-src：工作区源码中已经存在的 ROS 包不从 apt 重复安装。
- --rosdistro humble：按照 ROS Humble 的规则解析。
- -r：某个依赖失败时继续检查其他依赖。
- -y：自动确认安装。

和其他工具的关系是：
```
package.xml
   ↓
rosdep：安装系统/ROS依赖
   ↓
colcon：组织并构建整个ROS工作区
   ↓
CMake/ament：编译具体C++包
```

# 编译C/C++文件
```
bash tools/build.sh # 快捷脚本
```

```
#!/usr/bin/env bash # shebang
set -euo pipefail # 开启严格错误处理

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" # 自动定位项目根目录
ros_distro="${ROS_DISTRO:-humble}" # 确定 ROS 版本

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    echo "ROS2 setup missing: /opt/ros/${ros_distro}/setup.bash" >&2
    exit 1
fi

source "/opt/ros/${ros_distro}/setup.bash" # 加载 ROS 基础环境
cd "$workspace_dir"

colcon build \
    --base-paths src \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    "$@"
```

# 启动

```
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch rm_aa_bringup bringup.launch.py source:=camera device:=0 hardware:=true
```

从视频目录读取：

```
ros2 launch rm_aa_bringup bringup.launch.py \
  source:=video \
  video:=/absolute/path/to/videos \
  mock_gimbal:=true \
  hardware:=false \
  web:=true
```

如需循环，在 [perception.yaml (line 20)](/home/krazyk/Desktop/RM_AA/src/rm_aa_bringup/config/perception.yaml:20) 设置：`loop_video: true`