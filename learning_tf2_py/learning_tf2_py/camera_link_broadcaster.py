import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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


def main(args=None):
    rclpy.init(args=args)
    node = Node('camera_link_broadcaster')

    broadcaster = StaticTransformBroadcaster(node)

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'turtle1'
    t.child_frame_id = 'camera_link'

    # Camera is mounted 0.3 units in front of the robot
    t.transform.translation.x = 0.3
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    q = quaternion_from_euler(0.0, 0.0, 0.0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster.sendTransform(t)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()