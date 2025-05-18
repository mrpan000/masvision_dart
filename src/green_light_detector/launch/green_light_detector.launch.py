from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('green_light_detector'),
        'config',
        'green_light.yaml'
    )

    return LaunchDescription([
        Node(
            package='green_light_detector',
            executable='green_light_detector',
            name='green_light_detector_node',
            parameters=[config],
            output='screen'
        )
    ])