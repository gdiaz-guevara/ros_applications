from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    ),
    DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rasbot_description'),
            'urdf',
            'rasbot.urdf.xacro'
        ]),
    ),
]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('urdf_launch'),
                'launch',
                'display.launch.py'
            ]),
            launch_arguments={
                'urdf_package': 'rasbot_description',
                'urdf_package_path': LaunchConfiguration('model'),
                'jsp_gui': LaunchConfiguration('gui')
            }.items()
        )
    )

    return ld