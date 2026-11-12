"""NAO + MoveIt 2 in Gazebo.

    ros2 launch nao_moveit_config nao_moveit.launch.py

It starts three things:
  1. the NAO in Gazebo with its ros2_control controllers (nao_gz_controller)
  2. move_group: the MoveIt "brain" (planning, IK, collision checking, execution)
  3. RViz with the MotionPlanning panel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    nao_urdf = os.path.join(get_package_share_directory('nao_gz_controller'),
                            'urdf', 'nao_gazebo.urdf.xacro')

    # Collect every MoveIt parameter from this package
    moveit_config = (
        MoveItConfigsBuilder('nao', package_name='nao_moveit_config')
        .robot_description(file_path=nao_urdf)                  # URDF: the robot
        .robot_description_semantic(file_path='config/nao.srdf')  # SRDF: groups, poses, collisions
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])                 # default OMPL settings
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # 1. Robot in Gazebo + controllers (from the ros2_control module)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nao_gz_controller'), 'launch', 'nao_gz.launch.py')))

    # 2. move_group
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 3. RViz with MoveIt's MotionPlanning panel
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(get_package_share_directory('nao_moveit_config'),
                                      'rviz', 'moveit.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {'use_sim_time': True}],
    )

    # Give Gazebo and the controllers a few seconds before MoveIt starts
    return LaunchDescription([gazebo, TimerAction(period=8.0, actions=[move_group, rviz])])
