import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mas_serial_driver'), 'config', 'serial_driver.yaml')

    mas_serial_driver_node = Node(
        package='mas_serial_driver',
        executable='mas_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([mas_serial_driver_node])
