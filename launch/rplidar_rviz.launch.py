import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Specify the name of the package, path to rviz-config file and config-file name
    pkg_name = 'rplidar_utilities'
    rviz_config_subpath = 'rviz'
    rviz_config_filename = 'rplidar_utilities.rviz'

    # Determine full file
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name),
                                    rviz_config_subpath, rviz_config_filename)

    node_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    ld = LaunchDescription()

    ld.add_action(node_rviz2_cmd)

    return ld
