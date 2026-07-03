from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="Porta serial do ArbotiX-M"),
        Node(
            package="widowx_control",
            executable="widowx_driver",
            name="widowx_driver",
            parameters=[{"port": port}],
            output="screen",
        ),
        Node(
            package="widowx_control",
            executable="widowx_gui",
            name="widowx_gui",
            output="screen",
        ),
    ])
