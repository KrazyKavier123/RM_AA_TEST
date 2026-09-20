#!/usr/bin/env python3
"""Linux ROS2 integration test. Runs the real C++ node against a PTY, never real hardware."""
import os
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(80 + os.getpid() % 40)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from rm_aa_hardware.msg import GimbalFeedback, TargetCommand

NODE = sys.argv.pop(1)


class HardwarePty(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.client = rclpy.create_node('hardware_pty_test')
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.buffer = bytearray()
        self.packets = []
        self.feedback = self.joints = self.transform = None
        self.connected = False
        self.last_rx = 0.0
        self.mcu_status = 5
        self.client.create_subscription(Bool, '/hardware/connected', self.on_link, 1)
        self.client.create_subscription(GimbalFeedback, '/gimbal/feedback', self.on_feedback, qos_profile_sensor_data)
        self.client.create_subscription(JointState, '/gimbal/joint_states', self.on_joints, qos_profile_sensor_data)
        self.client.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.command = self.client.create_publisher(TargetCommand, '/gimbal/target_command', 1)
        self.trajectory = self.client.create_publisher(JointTrajectory, '/gimbal/trajectory', 1)
        source = 'trajectory' if 'trajectory' in self._testMethodName else 'target_command'
        self.log = tempfile.TemporaryFile(mode='w+b')
        self.process = subprocess.Popen([
            NODE, '--ros-args', '-p', f'serial.device:={os.ttyname(self.slave)}',
            '-p', f'command_source:={source}', '-p', 'trajectory_target_id:=3',
            '-p', 'enable_fire:=true'], stdout=self.log, stderr=subprocess.STDOUT)
        # addCleanup also runs if discovery/initialization fails in setUp.
        self.addCleanup(self.cleanup)
        self.wait_for(lambda: self.connected, timeout=10)
        self.wait_for(lambda: self.feedback is not None and self.feedback.velocity_valid)
        publisher = self.trajectory if source == 'trajectory' else self.command
        self.wait_for(lambda: publisher.get_subscription_count() > 0)

    def cleanup(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        os.close(self.master)
        os.close(self.slave)
        self.log.close()
        self.client.destroy_node()
        rclpy.shutdown()

    def on_link(self, msg):
        self.connected = msg.data

    def on_feedback(self, msg):
        self.feedback = msg

    def on_joints(self, msg):
        self.joints = msg

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'gimbal_attitude':
                self.transform = transform

    def pump(self, feed=True, publish=None):
        if self.process.poll() is not None:
            self.log.seek(0)
            self.fail(self.log.read().decode(errors='replace'))
        if time.monotonic() - self.last_rx > 0.02:
            if feed:
                frame = bytearray(64)
                frame[0], frame[63], frame[13] = 0x71, 0x4c, self.mcu_status
                struct.pack_into('<fff', frame, 1, 0.25, -0.1, 0.03)
                struct.pack_into('<f', frame, 20, 0.04)
                # Prefix noise and split the frame: the last full frame must decode immediately.
                os.write(self.master, b'\x01\x02' + frame[:17])
                os.write(self.master, frame[17:])
            if publish:
                publish()
            self.last_rx = time.monotonic()
        rclpy.spin_once(self.client, timeout_sec=0.005)
        try:
            self.buffer.extend(os.read(self.master, 4096))
        except BlockingIOError:
            pass
        while len(self.buffer) >= 64:
            if self.buffer[0] == 0x71 and self.buffer[63] == 0x4c:
                self.packets.append(bytes(self.buffer[:64]))
                del self.buffer[:64]
            else:
                del self.buffer[0]

    def wait_for(self, predicate, timeout=3, feed=True, publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(feed, publish)
            if predicate():
                return
        self.log.seek(0)
        self.fail('Condition timed out\n' + self.log.read().decode(errors='replace'))

    def send_command(self):
        msg = TargetCommand()
        msg.header.stamp = self.client.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_base'
        msg.target_id, msg.tracking_state = 3, TargetCommand.STABLE
        msg.target_yaw, msg.target_pitch = 0.45, 0.12
        msg.yaw_velocity, msg.pitch_velocity = 0.2, -0.1
        msg.distance_m, msg.center_x_m, msg.center_y_m = 3.0, 1.25, -0.5
        msg.fire_allowed = True
        self.command.publish(msg)

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.client.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_base'
        # Deliberately reverse order to exercise name mapping.
        msg.joint_names = ['gimbal_pitch_joint', 'gimbal_yaw_joint']
        for ns in (0, 100000000):
            point = JointTrajectoryPoint()
            point.positions = [0.12, 0.45]
            point.velocities = [-0.1, 0.2]
            point.time_from_start.nanosec = ns
            msg.points.append(point)
        self.trajectory.publish(msg)

    def last_is_target(self):
        return bool(self.packets) and self.packets[-1][15] == 3

    def last_is_safe(self):
        return bool(self.packets) and self.packets[-1][15] == 0 and self.packets[-1][44] == 0

    def test_feedback_full_command_and_watchdogs(self):
        self.wait_for(lambda: self.joints is not None and len(self.joints.velocity) == 3 and self.transform is not None)
        self.assertAlmostEqual(self.feedback.yaw, 0.25, places=5)
        self.assertAlmostEqual(self.feedback.pitch, -0.1, places=5)
        self.assertAlmostEqual(self.feedback.roll, 0.03, places=5)
        self.assertAlmostEqual(self.feedback.prediction_bias_s, 0.04, places=5)
        self.assertEqual(self.joints.name[2], 'gimbal_roll_joint')
        q = self.transform.transform.rotation
        self.assertAlmostEqual(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, places=6)
        self.wait_for(self.last_is_target, publish=self.send_command)
        packet = self.packets[-1]
        self.assertEqual(struct.unpack_from('<H', packet, 61)[0], 1)
        self.assertEqual(packet[44], 1)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 32)[0], 1250)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 36)[0], -500)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 45)[0], 0.45, places=5)
        self.wait_for(self.last_is_safe)  # RX continues but command expires.
        self.assertAlmostEqual(struct.unpack_from('<f', self.packets[-1], 45)[0], 0.25, places=5)
        self.wait_for(self.last_is_target, publish=self.send_command)
        self.wait_for(self.last_is_safe, feed=False, publish=self.send_command)  # RX watchdog.
        self.mcu_status = 1
        for _ in range(50):
            self.pump(publish=self.send_command)
        self.assertTrue(self.last_is_safe())

    def test_trajectory_maps_axes_and_never_fires(self):
        self.wait_for(self.last_is_target, publish=self.send_trajectory)
        packet = self.packets[-1]
        self.assertEqual(struct.unpack_from('<H', packet, 61)[0], 2)
        self.assertEqual(packet[44], 0)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 45)[0], 0.45, places=5)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 49)[0], 0.12, places=5)
        self.assertAlmostEqual(struct.unpack_from('<f', packet, 53)[0], 0.2, places=5)
        self.wait_for(self.last_is_safe)


if __name__ == '__main__':
    unittest.main()
