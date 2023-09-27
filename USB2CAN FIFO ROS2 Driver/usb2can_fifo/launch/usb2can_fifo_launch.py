# usb2can_fifo_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('usb2can_fifo')

    return LaunchDescription([
        Node(
            package='usb2can_fifo',
            executable='usb2can_fifo_node',
            name='usb2can_fifo_node',
            output='screen',
            parameters=[package_dir + '/yaml/config.yaml'],  # Load the YAML file
        ),
    ])