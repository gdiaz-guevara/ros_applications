#!/usr/bin/env python3
"""Drive a 1 m square (open loop) by publishing TwistStamped to the diff_drive_controller.

    ros2 run p3dx_ros2_control square_drive.py
    ros2 run p3dx_ros2_control square_drive.py --ros-args -p side:=2.0 -p speed:=0.3

At the end it prints where the controller's odometry thinks the robot is.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SquareDrive(Node):

    def __init__(self):
        super().__init__('square_drive')
        self.side = self.declare_parameter('side', 1.0).value
        self.speed = self.declare_parameter('speed', 0.25).value
        self.turn_speed = self.declare_parameter('turn_speed', 0.5).value
        # In ROS 2 Jazzy the diff_drive_controller listens to TwistStamped on ~/cmd_vel
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 lambda m: setattr(self, 'odom', m), 10)

        # Plan: (linear, angular, duration) segments
        self.plan = []
        for _ in range(4):
            self.plan.append((self.speed, 0.0, self.side / self.speed))
            self.plan.append((0.0, self.turn_speed, (math.pi / 2) / self.turn_speed))
        self.plan.append((0.0, 0.0, 0.5))
        self.segment = 0
        self.segment_start = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.step)   # 20 Hz (> 1 / cmd_vel_timeout)

    def step(self):
        if self.segment >= len(self.plan):
            self.timer.cancel()
            if self.odom:
                p = self.odom.pose.pose.position
                self.get_logger().info(f'Done. Odometry says x={p.x:.3f} y={p.y:.3f}')
            raise SystemExit
        lin, ang, duration = self.plan[self.segment]
        elapsed = (self.get_clock().now() - self.segment_start).nanoseconds * 1e-9
        if elapsed >= duration:
            self.segment += 1
            self.segment_start = self.get_clock().now()
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SquareDrive()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
