import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    ras_navigation_pkg = get_package_share_directory(
        "ras_navigation")
    launch_dir = os.path.join(ras_navigation_pkg, 'launch')
    params_dir = os.path.join(ras_navigation_pkg, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # use_rviz = LaunchConfiguration('use_rviz')
    # use_mapviz = LaunchConfiguration('use_mapviz')

    # declare_use_rviz_cmd = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value='False',
    #     description='Whether to start RVIZ')

    # declare_use_mapviz_cmd = DeclareLaunchArgument(
    #     'use_mapviz',
    #     default_value='False',
    #     description='Whether to start mapviz')


    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
    #     condition=IfCondition(use_rviz)
    # )

    # mapviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'mapviz.launch.py')),
    #     condition=IfCondition(use_mapviz)
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(rviz_cmd)
    # ld.add_action(declare_use_mapviz_cmd)
    # ld.add_action(mapviz_cmd)

    return ld