
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys
import subprocess

# Launch file for misao process from
# kurome. Mostly just provides the loading
# of parameters from a config file

def generate_launch_description():

    # get config file
    config = os.path.join(
            get_package_share_directory('odrive_diff_drive'),
            'config',
            'diff_drive_config.yaml'
    )

    # the node
    diff_drive_node = launch_ros.actions.Node(
            package='odrive_diff_drive',
            executable='diff_drive',
            namespace='odrive_diff_drive',
            name='diff_drive',
            parameters=[config]
    )

    return launch.LaunchDescription([
        diff_drive_node
    ])
