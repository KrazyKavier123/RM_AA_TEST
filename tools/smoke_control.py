#!/usr/bin/env python3
"""ROS2 integration smoke: actual synthetic vision -> TF -> EKF -> MPC trajectory."""
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory


def main():
    rclpy.init()
    node = Node('control_smoke_probe')
    seen = {'observations': 0, 'commands': 0}

    def observations(msg):
        if msg.poses:
            seen['observations'] += 1

    def command(msg):
        assert msg.joint_names == ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        assert len(msg.points) == 25
        last = 0.0
        for point in msg.points:
            assert len(point.positions) == len(point.velocities) == len(point.accelerations) == 2
            assert all(math.isfinite(x) for x in point.positions + point.velocities + point.accelerations)
            assert abs(point.accelerations[0]) <= 6.0 + 1e-8
            assert abs(point.accelerations[1]) <= 4.0 + 1e-8
            stamp = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            assert stamp > last
            last = stamp
        seen['commands'] += 1

    node.create_subscription(PoseArray, '/observations', observations, qos_profile_sensor_data)
    node.create_subscription(JointTrajectory, '/gimbal/trajectory', command, 5)
    process = subprocess.Popen([
        'ros2', 'launch', 'rm_aa_bringup', 'tracking.launch.py',
        'source:=synthetic', 'mock_gimbal:=true', 'web:=false',
    ])
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and seen['commands'] < 10:
            if process.poll() is not None:
                raise RuntimeError('Tracking launch exited early')
            rclpy.spin_once(node, timeout_sec=0.1)
        assert seen['observations'] > 0 and seen['commands'] >= 10, seen
        print('PASS: actual vision/control ROS topics, finite trajectory, acceleration bounds', seen)
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
