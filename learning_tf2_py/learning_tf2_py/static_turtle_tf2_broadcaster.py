import math
import sys
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


def publish_static_transform(node, args):
    t = TransformStamped()

    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = args[1]

    t.transform.translation.x = float(args[2])
    t.transform.translation.y = float(args[3])
    t.transform.translation.z = float(args[4])

    q = quaternion_from_euler(
        float(args[5]), float(args[6]), float(args[7])
    )

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(t)


def main():
    if len(sys.argv) != 8:
        print("Usage: static_turtle_tf2_broadcaster child x y z roll pitch yaw")
        sys.exit(1)

    if sys.argv[1] == 'world':
        print('Child frame cannot be "world"')
        sys.exit(2)

    rclpy.init()
    node = Node('static_turtle_tf2_broadcaster')

    publish_static_transform(node, sys.argv)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()