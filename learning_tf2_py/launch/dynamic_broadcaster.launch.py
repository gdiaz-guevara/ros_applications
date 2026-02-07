from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='dynamic_broadcaster',
            parameters=[{'turtlename': 'turtle1'}]
        )
    ])