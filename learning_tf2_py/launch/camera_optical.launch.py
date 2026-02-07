from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='turtle1_broadcaster',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        Node(
            package='learning_tf2_py',
            executable='camera_link_broadcaster',
            name='camera_link_broadcaster'
        ),
        Node(
            package='learning_tf2_py',
            executable='camera_optical_broadcaster',
            name='camera_optical_broadcaster'
        ),
    ])