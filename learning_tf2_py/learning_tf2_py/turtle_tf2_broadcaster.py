import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0

    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)

    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


def handle_pose(msg, node, broadcaster, turtlename):
    t = TransformStamped()

    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = turtlename

    # Translation
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    # Rotation (only yaw)
    q = quaternion_from_euler(0.0, 0.0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = Node('turtle_tf2_frame_publisher')

    turtlename = node.declare_parameter(
        'turtlename', 'turtle').get_parameter_value().string_value

    broadcaster = TransformBroadcaster(node)

    node.create_subscription(
        Pose,
        f'/{turtlename}/pose',
        lambda msg: handle_pose(msg, node, broadcaster, turtlename),
        1
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()