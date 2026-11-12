from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_jackal_gazebo = FindPackageShare('jackal_gazebo')

    world_name_arg = DeclareLaunchArgument('world_name', default_value='empty.sdf')
    entity_arg = DeclareLaunchArgument('entity', default_value='jackal')
    x_arg = DeclareLaunchArgument('x', default_value='0.5')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    spawn_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_jackal_gazebo, 'launch', 'spawn_world.launch.py'])),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name')
        }.items()
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_jackal_gazebo, 'launch', 'spawn_robot.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'entity': LaunchConfiguration('entity'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    return LaunchDescription([
        world_name_arg,
        entity_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_world_cmd,
        spawn_robot_cmd
    ])