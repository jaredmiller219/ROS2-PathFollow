"""Provide minimal ROS interfaces when running tests outside a ROS install."""

import importlib.util
import sys
import types


if importlib.util.find_spec("rclpy") is None:
    class _Vector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Twist:
        def __init__(self):
            self.linear = _Vector3()
            self.angular = _Vector3()

    class _Node:
        def __init__(self, name):
            self.name = name
            self.logger = types.SimpleNamespace(
                info=lambda message: None,
                warning=lambda message: None,
                error=lambda message: None,
            )

        def create_subscription(self, *args):
            return object()

        def create_publisher(self, *args):
            return types.SimpleNamespace(messages=[], publish=lambda message: None)

        def create_timer(self, period, callback):
            return types.SimpleNamespace(
                period=period,
                callback=callback,
                cancelled=False,
                cancel=lambda: None,
            )

        def count_subscribers(self, topic):
            return 0

        def count_publishers(self, topic):
            return 0

        def destroy_timer(self, timer):
            pass

        def destroy_node(self):
            pass

        def get_logger(self):
            return self.logger

    class _SingleThreadedExecutor:
        def add_node(self, node):
            pass

        def spin(self):
            pass

        def shutdown(self):
            pass

    class _Odometry:
        def __init__(self):
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=_Vector3(),
                    orientation=types.SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0,
                    ),
                )
            )

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.qos = types.SimpleNamespace(qos_profile_sensor_data=object())

    node_module = types.ModuleType("rclpy.node")
    node_module.Node = _Node
    executors_module = types.ModuleType("rclpy.executors")

    class _ExternalShutdownException(Exception):
        pass

    executors_module.ExternalShutdownException = _ExternalShutdownException
    executors_module.SingleThreadedExecutor = _SingleThreadedExecutor

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _Twist
    geometry_msgs.msg = geometry_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
    sensor_msgs.msg = sensor_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = _Odometry
    nav_msgs.msg = nav_msgs_msg

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": node_module,
            "rclpy.executors": executors_module,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
        }
    )
