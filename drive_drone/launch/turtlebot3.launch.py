#!/usr/bin/env python3

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Set the Turtlebot model
    os.environ["TURTLEBOT3_MODEL"] = "waffle_pi"

    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    
    # Wait for 2 seconds to allow the model to be set
    time.sleep(2)

    tb3_bringup= IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
            ),

        )
    
    nodes_to_run = [
        tb3_bringup,
    ]

    return LaunchDescription(nodes_to_run)