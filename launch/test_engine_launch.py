#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the can_bridge_node
    can_bridge_node = Node(
        package='can_interfaces',       # Replace with your can_bridge package name
        executable='can_bridge_node',   # Replace with your can_bridge executable name
        name='can_bridge_node'
    )

    # Define the h20pro node
    h20pro_node = Node(
        package='can_interfaces',           # Replace with your h20pro package name
        executable='h20pro_node',       # Replace with your h20pro executable name
        name='h20pro_node'
    )

    """
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )
    """

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch/rosbridge_websocket_launch.xml",
            )
        )
    )

    npm_app = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd = ['npm', 'start'],
                cwd = os.path.join(os.getcwd(), 'machpilot-gui'),
                shell = True,
                output = 'screen'
            )

        ]
    )

    return LaunchDescription([
        can_bridge_node,
        TimerAction(period=5.0, actions=[h20pro_node]),
        TimerAction(period=10.0, actions=[rosbridge_launch]),
        npm_app
    ])
