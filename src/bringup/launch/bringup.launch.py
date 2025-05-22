import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 路径
    hik_camera_config = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')
    hik_camera_info = 'package://hik_camera/config/camera_info.yaml'

    green_light_config = os.path.join(
        get_package_share_directory('green_light_detector'), 'config', 'green_light.yaml')

    serial_driver_config = os.path.join(
        get_package_share_directory('mas_serial_driver'), 'config', 'serial_driver.yaml')

    return LaunchDescription([
        Node(
           package='hik_camera',
           executable='hik_camera_node',
           output='screen',
           emulate_tty=True,
           parameters=[hik_camera_config, {'camera_info_url': hik_camera_info}],
        ),
        Node(
            package='green_light_detector',
            executable='green_light_detector',
            name='green_light_detector_node',
            parameters=[green_light_config],
            output='screen'
        ),
        Node(
            package='mas_serial_driver',
            executable='mas_serial_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[serial_driver_config],
        ),
    ])