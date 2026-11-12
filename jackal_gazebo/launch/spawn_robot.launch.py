import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file
from nav2_common.launch import ReplaceString

def get_robot_description():
    pkg_jackal_gazebo = get_package_share_directory('jackal_gazebo')
    pkg_jackal_description = get_package_share_directory('jackal_description')
    robot_description_path = os.path.join(pkg_jackal_gazebo, 'urdf', 'jackal_gz.urdf.xacro')
    
    mappings = {}
    robot_description_config = process_file(robot_description_path, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    robot_desc = robot_desc.replace(
        'package://jackal_description/', f'file://{pkg_jackal_description}/'
    )
    return robot_desc

def generate_launch_description():
    pkg_jackal_gazebo = get_package_share_directory('jackal_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    entity = LaunchConfiguration('entity')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            'robot_description': get_robot_description(),
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', entity,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', '0',
            '-P', '0',
            '-Y', yaw,
        ],
        output='screen',
    )

    bridge_config_file_path = os.path.join(pkg_jackal_gazebo, 'config', 'bridge_config.yaml')
    bridge_config = ReplaceString(
        source_file=bridge_config_file_path,
        replacements={'<entity>': entity},
    )

    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
        name='clock_bridge'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('entity', default_value='jackal'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        rsp_node,
        spawn_node,
        parameter_bridge_node,
        clock_bridge_node
    ])