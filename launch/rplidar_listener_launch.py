#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# #from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_name = LaunchConfiguration('topic_name', default='scan')
    skip_ranges = LaunchConfiguration('skip_ranges', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'topic_name',
            default_value=topic_name,
            description='Specifying topic name of LaserScan message'),

        DeclareLaunchArgument(
            'skip_ranges',
            default_value=skip_ranges,
            description='Skip output of all range-values. Only header info will be displayed.'),

        Node(
            package='rplidar_utilities',
            executable='rplidar_listener',
            name='rplidar_listener',
            parameters=[{'topic_name': topic_name,
                         'skip_ranges':skip_ranges}],
            output='screen'),
    ])
