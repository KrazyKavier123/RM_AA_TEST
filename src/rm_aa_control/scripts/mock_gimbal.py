#!/usr/bin/env python3
"""Fixed camera / zero feedback source for interface demo; does not follow commands."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster


class MockGimbal(Node):
    def __init__(self):
        super().__init__('mock_gimbal')
        self.pub = self.create_publisher(JointState, '/gimbal/joint_states', 5)
        self.tf = StaticTransformBroadcaster(self)
        base = TransformStamped()
        base.header.stamp = self.get_clock().now().to_msg()
        base.header.frame_id = 'odom'
        base.child_frame_id = 'gimbal_base'
        base.transform.rotation.w = 1.0
        camera = TransformStamped()
        camera.header.stamp = self.get_clock().now().to_msg()
        camera.header.frame_id = 'gimbal_base'
        camera.child_frame_id = 'camera_optical_frame'
        # P-axis -> camera, converted from optical [right, down, forward]
        # [0.06711, 0, 0.0515] m to gimbal [forward, left, up].
        camera.transform.translation.x = -0.0515
        camera.transform.translation.y = 0.06711
        # optical right/down/forward -> base forward/left/up
        camera.transform.rotation.x = -0.5
        camera.transform.rotation.y = 0.5
        camera.transform.rotation.z = -0.5
        camera.transform.rotation.w = 0.5
        self.tf.sendTransform([base, camera])
        self.timer = self.create_timer(0.02, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gimbal_yaw_joint', 'gimbal_pitch_joint']
        msg.position = [0.0, 0.0]
        msg.velocity = [0.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MockGimbal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
