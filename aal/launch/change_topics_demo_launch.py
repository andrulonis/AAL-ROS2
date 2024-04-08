from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='aal',
            executable='adaptation_layer',
        ),
        Node(
            package='aal',
            executable='demo_publisherB',
        ),
        Node(
            package='aal',
            executable='demo_subscriber',
        ),
        Node(
            package='aal',
            executable='demo_change_topics',
        ),
    ])