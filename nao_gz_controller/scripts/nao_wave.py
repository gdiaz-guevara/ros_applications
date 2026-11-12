#!/usr/bin/env python3
"""Make the NAO wave with its right arm and nod with its head.

Sends FollowJointTrajectory goals to two joint_trajectory_controllers and
waits for the result.

    ros2 run nao_gz_controller nao_wave.py
"""
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RIGHT_ARM = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
HEAD = ['HeadYaw', 'HeadPitch']


def make_trajectory(joint_names, waypoints):
    """waypoints = [(seconds_from_start, [positions...]), ...]"""
    traj = JointTrajectory()
    traj.joint_names = joint_names
    for t, positions in waypoints:
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points.append(point)
    return traj


class NaoWave(Node):

    def __init__(self):
        super().__init__('nao_wave')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/joint_trajectory_controller_right_arm/follow_joint_trajectory')
        self.head = ActionClient(self, FollowJointTrajectory,
                                 '/joint_trajectory_controller_head/follow_joint_trajectory')

    def send(self, client, trajectory):
        client.wait_for_server()
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        return handle.get_result_async()

    def run(self):
        # Arm up (shoulder pitch negative = forward/up), then wave the elbow 3 times
        wave = [(1.5, [-1.2, -0.3, 1.2, 0.5, 0.0])]
        t = 1.5
        for _ in range(3):
            t += 0.6
            wave.append((t, [-1.2, -0.3, 1.2, 1.4, 0.0]))
            t += 0.6
            wave.append((t, [-1.2, -0.3, 1.2, 0.5, 0.0]))
        wave.append((t + 1.5, [1.5, -0.1, 1.2, 0.3, 0.0]))   # arm down

        nod = [(1.0, [0.0, 0.3]), (2.0, [0.0, -0.3]), (3.0, [0.6, 0.0]),
               (4.0, [-0.6, 0.0]), (5.0, [0.0, 0.0])]

        self.get_logger().info('Waving and nodding...')
        arm_result = self.send(self.arm, make_trajectory(RIGHT_ARM, wave))
        head_result = self.send(self.head, make_trajectory(HEAD, nod))
        for name, res in (('right arm', arm_result), ('head', head_result)):
            if res is not None:
                rclpy.spin_until_future_complete(self, res)
                code = res.result().result.error_code
                self.get_logger().info(f'{name}: error_code={code} (0 = SUCCESSFUL)')


def main():
    rclpy.init()
    node = NaoWave()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
