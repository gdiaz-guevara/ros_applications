"""Run scripts/nao_moveit_py.py with all the MoveIt parameters it needs.
(nao_moveit.launch.py must be running.)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    nao_urdf = os.path.join(get_package_share_directory('nao_gz_controller'),
                            'urdf', 'nao_gazebo.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('nao', package_name='nao_moveit_config')
        .robot_description(file_path=nao_urdf)
        .robot_description_semantic(file_path='config/nao.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .moveit_cpp(file_path='config/moveit_py.yaml')   # extra params for moveit_py
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(package='nao_moveit_config', executable='nao_moveit_py.py', output='screen',
             parameters=[moveit_config.to_dict(), {'use_sim_time': True}]),
    ])
