from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('svo_ros'))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/svo.launch.py')),
        launch_arguments={'pipeline':'stereo','params':str(share/'param/frontend_imu/euroc_stereo_imu.yaml'),'calib_file':str(share/'param/calib/euroc_stereo.yaml')}.items())])
