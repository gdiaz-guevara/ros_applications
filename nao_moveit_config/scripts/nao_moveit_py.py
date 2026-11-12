#!/usr/bin/env python3
"""Plan and execute NAO arm motions with the MoveIt 2 Python API (moveit_py).

Start the simulation + MoveIt first:
    ros2 launch nao_moveit_config nao_moveit.launch.py
Then:
    ros2 launch nao_moveit_config nao_moveit_py.launch.py
"""
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.logging import get_logger

log = get_logger('nao_moveit_py')


def plan_and_execute(robot, arm, label):
    """Plan from the current state to the goal already set in `arm` and execute it."""
    log.info(f'Planning: {label}')
    result = arm.plan()
    if not result:
        log.error(f'  planning FAILED: {label}')
        return False
    log.info('  plan found, executing...')
    robot.execute(result.trajectory, controllers=[])
    return True


def main():
    rclpy.init()
    # MoveItPy reads all MoveIt parameters given by the launch file
    nao = MoveItPy(node_name='nao_moveit_py')
    right_arm = nao.get_planning_component('right_arm')
    robot_model = nao.get_robot_model()
    log.info('MoveItPy ready')
    time.sleep(3.0)   # let the action clients connect to the trajectory controllers

    # 1) Go to a NAMED pose defined in the SRDF
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(configuration_name='arm_forward')
    plan_and_execute(nao, right_arm, 'named pose "arm_forward"')
    time.sleep(1.0)

    # 2) Go to a JOINT goal (radians)
    goal = RobotState(robot_model)
    goal.set_joint_group_positions('right_arm', [-0.5, -0.6, 1.0, 1.0, 0.0])
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(robot_state=goal)
    plan_and_execute(nao, right_arm, 'joint goal')
    time.sleep(1.0)

    # 3) Go to a CARTESIAN goal: put the hand at a point (x, y, z) in the Torso frame.
    #    The arm has 5 joints, so we ask inverse kinematics for the POSITION only
    #    (position_only_ik in kinematics.yaml) and then plan to that joint solution.
    target = Pose()
    target.position.x, target.position.y, target.position.z = 0.15, -0.10, 0.10
    target.orientation.w = 1.0
    ik_state = RobotState(robot_model)
    ik_state.set_to_default_values()
    if ik_state.set_from_ik('right_arm', target, 'RWrist', 1.0):
        right_arm.set_start_state_to_current_state()
        right_arm.set_goal_state(robot_state=ik_state)
        plan_and_execute(nao, right_arm, 'hand to (0.15, -0.10, 0.10)')
    else:
        log.error('No IK solution: the point is out of reach')
    time.sleep(1.0)

    # 4) Back home
    right_arm.set_start_state_to_current_state()
    right_arm.set_goal_state(configuration_name='home')
    plan_and_execute(nao, right_arm, 'named pose "home"')

    log.info('Done')
    nao.shutdown()     # stop MoveItPy cleanly BEFORE rclpy (avoids a crash at exit)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
