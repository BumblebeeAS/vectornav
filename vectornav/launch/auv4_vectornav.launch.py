import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():

    launch_file_path = os.path.join(
        get_package_share_directory("vectornav"), "launch", "vectornav.launch.py"
    )

    vectornav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
    )

    group_action = GroupAction(
        actions=[
            PushRosNamespace("auv4"),
            SetRemap(src="vectornav/imu", dst="vnav/imu_ned"),
            SetRemap(src="vectornav/magnetic", dst="vnav/mag"),
            SetRemap(src="vectornav/pressure", dst="vnav/pressure"),
            SetRemap(src="vectornav/temperature", dst="vnav/temp"),
            vectornav_include,
        ]
    )

    return LaunchDescription([group_action])
