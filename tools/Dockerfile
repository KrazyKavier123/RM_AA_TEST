FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-yaml python3-opencv \
    libopencv-dev nlohmann-json3-dev ros-humble-cv-bridge ros-humble-ament-cmake-gtest \
    libeigen3-dev ros-humble-eigen3-cmake-module ros-humble-tf2-ros \
    ros-humble-nav-msgs ros-humble-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY src /ws/src
COPY licenses /ws/licenses
COPY tests /ws/tests
COPY tools /ws/tools
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON && \
    . /ws/install/setup.sh && \
    colcon test --return-code-on-test-failure --event-handlers console_direct+ && \
    colcon test-result --verbose && \
    python3 -m unittest discover -s tests -v && python3 tools/check_workspace.py
CMD ["bash", "-c", "source /ws/install/setup.bash && python3 tools/smoke_ros.py"]
