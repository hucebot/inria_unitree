from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que lee y publica JointStates
        Node(
            package='g1pilot',
            executable='ros_bridge',
            name='ros_bridge',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_to_livox_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '3.14159265',
                'mid360_link',
                'livox_frame'
            ]
        )
    ])
