"""ROS helpers owned by crazyflie_mapf."""

def qos(depth=1):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def timer(node, period, callback):
    from rclpy.clock import Clock, ClockType

    return node.create_timer(
        period, callback, clock=Clock(clock_type=ClockType.STEADY_TIME)
    )


def spin(node):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ros_time(node):
    return node.get_clock().now().nanoseconds * 1e-9
