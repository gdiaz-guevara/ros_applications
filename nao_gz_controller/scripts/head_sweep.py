#!/usr/bin/env python3
"""Move the NAO head with the head_position_controller (topic interface).

The controller must be ACTIVE first (it claims the same joints as
joint_trajectory_controller_head, so switch them):

    ros2 control switch_controllers --deactivate joint_trajectory_controller_head \
                                    --activate head_position_controller
    ros2 run nao_gz_controller head_sweep.py
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def main():
    rclpy.init()
    node = Node('head_sweep')
    pub = node.create_publisher(Float64MultiArray, '/head_position_controller/commands', 10)
    start = node.get_clock().now()

    def on_timer():
        t = (node.get_clock().now() - start).nanoseconds * 1e-9
        yaw = 0.8 * math.sin(0.5 * t)          # look left / right
        pitch = 0.2 * math.sin(1.0 * t)        # look up / down
        pub.publish(Float64MultiArray(data=[yaw, pitch]))   # order = joints in the YAML

    node.create_timer(0.02, on_timer)   # 50 Hz
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
