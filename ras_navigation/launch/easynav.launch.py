from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ras_navigation = FindPackageShare('ras_navigation')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='costmap.rpp.params.yaml',
        description='Archivo de parámetros de EasyNav (indoor: costmap.rpp.params.yaml, outdoor: gps.params.yaml / mpc.params.yaml)'
    )

    playground_pkg_arg = DeclareLaunchArgument(
        'playground_pkg',
        default_value='jackal_gazebo',
        description='Package to use for the playground (e.g., jackal_gazebo, easynav_gazebo_sim)'
    )

    playground_launch_arg = DeclareLaunchArgument(
        'playground_launch',
        default_value='jackal_gazebo.launch.py',
        description='Launch file for the playground (e.g., jackal_gazebo.launch.py, easynav_gazebo_sim.launch.yaml)'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='turtlebot3_world.sdf',
        description='World file to load (indoor: turtlebot3_world.sdf, outdoor: urjc_excavation.world)'
    )

    entity_arg = DeclareLaunchArgument('entity', default_value='jackal')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    params_file_path = PathJoinSubstitution([
        pkg_ras_navigation, 
        'config', 
        LaunchConfiguration('config_file')
    ])

    playground_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('playground_pkg')), 
            'launch', 
            LaunchConfiguration('playground_launch')
        ])),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'entity': LaunchConfiguration('entity'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    easynav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ras_navigation, 'launch', 'easynav_core.launch.py'])),
        launch_arguments={
            'params_file': params_file_path,
            'use_rviz': 'true'
        }.items()
    )

    return LaunchDescription([
        config_file_arg,
        playground_pkg_arg,
        playground_launch_arg,
        world_name_arg,
        entity_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        playground_cmd,
        easynav_cmd
    ])