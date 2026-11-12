"""Same robot, same controllers, NO simulator: mock hardware.

    ros2 launch p3dx_ros2_control p3dx_mock.launch.py

mock_components/GenericSystem copies every command to the matching state
(velocity command -> velocity state, integrated into position). It is how you
test controllers and launch files before connecting the real robot.
Here the controller_manager runs in its own process: ros2_control_node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    pkg = get_package_share_directory('p3dx_ros2_control')
    xacro_file = os.path.join(pkg, 'urdf', 'p3dx_ros2_control.urdf.xacro')
    robot_description = process_file(
        xacro_file, mappings={'use_mock_hardware': 'true'}).toprettyxml(indent='  ')
    controllers_yaml = os.path.join(pkg, 'config', 'p3dx_controllers.yaml')

    # Without Gazebo there is no gz_ros2_control plugin: the controller_manager
    # is started explicitly with ros2_control_node and the controllers YAML.
    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[controllers_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}], output='screen')
    spawner = Node(package='controller_manager', executable='spawner',
                   arguments=['joint_state_broadcaster', 'diff_drive_controller'],
                   output='screen')
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(pkg, 'config', 'p3dx_odom.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        control_node, rsp, rviz,
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawner])),
    ])
