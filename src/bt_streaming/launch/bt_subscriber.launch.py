from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bt_streaming'),
        'config',
        'subscriber_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='bt_streaming',
            executable='bt_subscriber_node',
            name='body_tracking_subscriber',
            parameters=[config],
            output='screen'
        )
    ])
