"""ROS imports stay optional so the Web service can still run in demo mode."""
import json


def numeric_fields(value, prefix=''):
    result = {}
    if isinstance(value, dict):
        for key, child in value.items():
            result.update(numeric_fields(child, f'{prefix}.{key}' if prefix else key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            result.update(numeric_fields(child, f'{prefix}.{index}'))
    elif isinstance(value, (int, float)):
        result[prefix] = value
    return result


def run_ros(hub, stop):
    node = context = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import CompressedImage, JointState
        from std_msgs.msg import String, Bool
        from nav_msgs.msg import Odometry
        from trajectory_msgs.msg import JointTrajectory
        from rm_aa_hardware.msg import GimbalFeedback, TargetCommand

        context = Context()
        rclpy.init(args=[], context=context)
        node = rclpy.create_node('rm_aa_monitor', context=context)
        qos = qos_profile_sensor_data

        def record(source, message):
            data = message_to_ordereddict(message)
            data.pop('header', None)
            hub.log(source, numeric_fields(data))

        def telemetry(message):
            try:
                data = json.loads(message.data)
                hub.log('vision', numeric_fields(data))
            except (ValueError, TypeError) as error:
                node.get_logger().warning(str(error))

        def trajectory(message):
            if message.points:
                data = message_to_ordereddict(message.points[0])
                hub.log('mpc.first_point', numeric_fields(data))

        subscriptions = [
            node.create_subscription(String, '/telemetry', telemetry, qos),
            node.create_subscription(CompressedImage, '/debug/image/compressed', lambda m: hub.image('image', m.data), qos),
            node.create_subscription(CompressedImage, '/debug/mask/compressed', lambda m: hub.image('mask', m.data), qos),
            node.create_subscription(GimbalFeedback, '/gimbal/feedback', lambda m: record('gimbal', m), qos),
            node.create_subscription(TargetCommand, '/gimbal/target_command', lambda m: record('command', m), qos),
            node.create_subscription(JointState, '/gimbal/joint_states', lambda m: record('joints', m), qos),
            node.create_subscription(Odometry, '/target/state', lambda m: record('ekf', m), qos),
            node.create_subscription(JointTrajectory, '/gimbal/trajectory', trajectory, qos),
            node.create_subscription(Bool, '/hardware/connected', lambda m: hub.log('hardware', {'connected': m.data}), qos),
        ]
        hub.ros = '已连接 ROS2'
        while not stop.is_set() and context.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as error:
        hub.ros = f'ROS 未连接：{error}'
    finally:
        if node:
            node.destroy_node()
        if context and context.ok():
            context.shutdown()
