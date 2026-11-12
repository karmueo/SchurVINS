"""Four-camera traditional SVO entry; requires a real four-camera calibration."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('svo_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('calib_file', description='SVO YAML containing four cameras'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/svo.launch.py')),
            launch_arguments={'pipeline':'array', 'calib_file':LaunchConfiguration('calib_file'), 'use_imu':'false'}.items())])
