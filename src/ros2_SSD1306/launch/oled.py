from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_SSD1306',
            namespace='oled1',
            executable='oled_executable',
            name='display'
        )
    ])