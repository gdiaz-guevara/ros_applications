import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from xacro import process_file
from nav2_common.launch import ReplaceString

ARGUMENTS = [
    DeclareLaunchArgument('world_name', default_value='empty.sdf'),
    DeclareLaunchArgument('ros_bridge', default_value='True'),
    DeclareLaunchArgument('initial_pose_x', default_value='0.5'),
    DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
    DeclareLaunchArgument('initial_pose_z', default_value='0.1'),
    DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    DeclareLaunchArgument('robot_description_topic', default_value='robot_description'),
    DeclareLaunchArgument('rsp_frequency', default_value='30.0'),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('entity', default_value='p3dx'),
]


def get_robot_description():
    pkg_p3dx_gazebo = get_package_share_directory('p3dx_gazebo')
    pkg_p3dx_description = get_package_share_directory('rasbot_description')
    robot_description_path = os.path.join(pkg_p3dx_gazebo, 'urdf', 'p3dx_gz.urdf.xacro')
    mappings = {}
    robot_description_config = process_file(robot_description_path, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    robot_desc = robot_desc.replace(
        'package://rasbot_description/', f'file://{pkg_p3dx_description}/'
    )
    return robot_desc


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_p3dx_gazebo = get_package_share_directory('p3dx_gazebo')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    world_name = LaunchConfiguration('world_name')
    ros_bridge = LaunchConfiguration('ros_bridge')
    world_path = PathJoinSubstitution([pkg_p3dx_gazebo,'worlds',world_name])
    bridge_config_file_path = os.path.join(pkg_p3dx_gazebo, 'config', 'bridge_config.yaml')
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            pkg_p3dx_gazebo,
            'models'
        )
    )

    ld.add_action(set_env_vars_resources)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_path],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    )

    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
            namespace='p3dx_gz_sim',
            condition=IfCondition(ros_bridge),
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rsp_frequency = LaunchConfiguration('rsp_frequency')
    
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'publish_frequency':  rsp_frequency,
                    'robot_description': get_robot_description(),
                }
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    )


    entity = LaunchConfiguration('entity')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    robot_description_topic = LaunchConfiguration('robot_description_topic')

    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', entity,
                '-topic', robot_description_topic,
                '-x', initial_pose_x,
                '-y', initial_pose_y,
                '-z', initial_pose_z,
                '-R', '0',
                '-P', '0',
                '-Y', initial_pose_yaw,
            ],
            output='screen',
        )
    )

    bridge_config = ReplaceString(
        source_file=bridge_config_file_path,
        replacements={'<entity>': entity},
    )

    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config
            }],
        )
    )

    return ld