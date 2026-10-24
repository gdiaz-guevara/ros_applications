from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ras_navigation = FindPackageShare('ras_navigation')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_ras_navigation, 'config', 'costmap.rpp.params.yaml']),
        description='EasyNav ROS 2 parameters file for costmap and mapping'
    )

    rviz_file_arg = DeclareLaunchArgument(
        'rviz_file',
        default_value='slam.rviz',
        description='Archivo de parámetros de EasyNav (indoor: costmap.rpp.params.yaml, outdoor: gps.params.yaml / mpc.params.yaml)'
    )
    
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_ras_navigation, 'rviz', LaunchConfiguration('rviz_file')]), 
        description='Rviz configuration file to use'
    )

    easynav_node = Node(
        package='easynav_system',
        executable='system_main',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        on_exit=EmitEvent(event=Shutdown())
    )

    timer_easynav = TimerAction(
        period=20.0,
        actions=[easynav_node]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        params_file_arg,
        use_rviz_arg,
        rviz_file_arg,
        rviz_config_arg,
        timer_easynav,
        rviz_node
    ])