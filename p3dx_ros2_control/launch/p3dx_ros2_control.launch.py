"""Pioneer 3-DX in Gazebo driven by ros2_control's diff_drive_controller.

    ros2 launch p3dx_ros2_control p3dx_ros2_control.launch.py
    ros2 launch p3dx_ros2_control p3dx_ros2_control.launch.py world_name:=turtlebot3_world.sdf x:=-2.0 y:=-0.5

Compared with p3dx_gazebo/p3dx_gz.launch.py:
  - there is NO DiffDrive / JointStatePublisher Gazebo plugin and no odom/tf/cmd_vel bridge:
    gz_ros2_control runs a controller_manager inside Gazebo and the controllers talk ROS 2 directly.
  - only /clock and /scan still go through ros_gz_bridge (sensors are not part of ros2_control here).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from xacro import process_file

ARGUMENTS = [
    DeclareLaunchArgument('world_name', default_value='empty.sdf',
                          description='World file inside p3dx_gazebo/worlds'),
    DeclareLaunchArgument('entity', default_value='p3dx'),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.1'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]


def robot_description():
    pkg_description = get_package_share_directory('rasbot_description')
    xacro_file = os.path.join(get_package_share_directory('p3dx_ros2_control'),
                              'urdf', 'p3dx_ros2_control.urdf.xacro')
    urdf = process_file(xacro_file).toprettyxml(indent='  ')
    return urdf.replace('package://rasbot_description/', f'file://{pkg_description}/')


def generate_launch_description():
    pkg_p3dx_gazebo = get_package_share_directory('p3dx_gazebo')

    # 1. Gazebo, RUNNING (-r): the controller_manager only updates while the simulation runs
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', PathJoinSubstitution(
                [pkg_p3dx_gazebo, 'worlds', LaunchConfiguration('world_name')])],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Only the clock and the LiDAR need the bridge now
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
    )

    # 3. robot_state_publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description()}],
    )

    # 4. Spawn
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', LaunchConfiguration('entity'), '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
    )

    # 5. Controllers: broadcaster first, then the diff drive controller
    jsb_spawner = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster'], output='screen')
    diff_spawner = Node(package='controller_manager', executable='spawner',
                        arguments=['diff_drive_controller'], output='screen')

    return LaunchDescription(ARGUMENTS + [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_p3dx_gazebo, 'models')),
        gazebo, bridge, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[diff_spawner])),
    ])
