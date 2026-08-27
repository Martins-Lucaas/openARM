from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.abspath('fruit_sorting_webots/worlds/Universal Robot OpenCV.wbt')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--batch', world_path],
            output='screen'
        )
    ])
