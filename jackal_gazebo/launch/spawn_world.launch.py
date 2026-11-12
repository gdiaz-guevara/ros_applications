import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_jackal_gazebo = FindPackageShare('jackal_gazebo')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Argumentos
    world_name_arg = DeclareLaunchArgument(
        'world_name', 
        default_value='empty.sdf',
        description='Name of the world file to load from the jackal_gazebo/worlds directory'
    )

    world_path = PathJoinSubstitution([pkg_jackal_gazebo, 'worlds', LaunchConfiguration('world_name')])

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([pkg_jackal_gazebo, 'models'])
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    return LaunchDescription([
        world_name_arg,
        set_env_vars_resources,
        gazebo_sim
    ])