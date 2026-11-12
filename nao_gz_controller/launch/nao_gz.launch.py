"""NAO humanoid in Gazebo Harmonic controlled with ros2_control.

    ros2 launch nao_gz_controller nao_gz.launch.py                          # ros2_control (default)
    ros2 launch nao_gz_controller nao_gz.launch.py use_ros2_control:=false  # Gazebo-native controllers

Startup order (ros2_control mode):
    Gazebo + robot_state_publisher + spawn
      -> (spawn finished)       joint_state_broadcaster
      -> (broadcaster active)   5 joint_trajectory_controllers (head, arms, legs)
                                + head_position_controller loaded INACTIVE
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from xacro import process_file

PKG = 'nao_gz_controller'

ARGUMENTS = [
    DeclareLaunchArgument('world_name', default_value='empty.sdf',
                          description='World file inside nao_gz_controller/worlds'),
    DeclareLaunchArgument('use_ros2_control', default_value='true',
                          choices=['true', 'false'],
                          description='true: gz_ros2_control + ROS 2 controllers, '
                                      'false: Gazebo-native position controllers'),
    DeclareLaunchArgument('entity', default_value='nao'),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.35'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]

TRAJECTORY_CONTROLLERS = [
    'joint_trajectory_controller_head',
    'joint_trajectory_controller_left_arm',
    'joint_trajectory_controller_right_arm',
    'joint_trajectory_controller_left_leg',
    'joint_trajectory_controller_right_leg',
]


def robot_description(use_ros2_control):
    """Run xacro and make the mesh paths absolute so Gazebo can find them."""
    pkg = get_package_share_directory(PKG)
    xacro_file = 'nao_gazebo.urdf.xacro' if use_ros2_control else 'nao_gz_native.urdf.xacro'
    urdf = process_file(os.path.join(pkg, 'urdf', xacro_file)).toprettyxml(indent='  ')
    return urdf.replace(f'package://{PKG}/', f'file://{pkg}/')


def launch_setup(context):
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context) == 'true'
    pkg = get_package_share_directory(PKG)
    controllers_yaml = os.path.join(pkg, 'config', 'nao_controllers.yaml')

    # 1. Gazebo (-r: start RUNNING; a paused simulation never activates controllers)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world_name')])],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Simulation clock -> ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 3. robot_state_publisher: /robot_description + TF from /joint_states
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description(use_ros2_control)}],
    )

    # 4. Spawn the robot from /robot_description
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', LaunchConfiguration('entity'), '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
    )

    actions = [gazebo, clock_bridge, rsp, spawn]

    if use_ros2_control:
        # 5. Controllers, started in order with event handlers
        jsb_spawner = Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )
        jtc_spawner = Node(
            package='controller_manager', executable='spawner',
            arguments=TRAJECTORY_CONTROLLERS + ['--param-file', controllers_yaml],
            output='screen',
        )
        head_position_spawner = Node(
            package='controller_manager', executable='spawner',
            arguments=['head_position_controller', '--inactive',
                       '--param-file', controllers_yaml],
            output='screen',
        )
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
            RegisterEventHandler(OnProcessExit(target_action=jsb_spawner,
                                               on_exit=[jtc_spawner, head_position_spawner])),
        ]
    else:
        # Gazebo-native controllers listen on gz topics; bridge the joint states only
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='joint_state_bridge',
            arguments=['/world/nao_world/model/nao/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
            remappings=[('/world/nao_world/model/nao/joint_state', '/joint_states')],
            output='screen',
        ))

    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    # Let Gazebo find the meshes/materials of this package
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(PKG))))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
