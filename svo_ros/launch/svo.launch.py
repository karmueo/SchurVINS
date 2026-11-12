"""Standard image/IMU topic entry for mono, stereo and four-camera SVO."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('svo_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('pipeline', default_value='stereo'),
        DeclareLaunchArgument('params', default_value=str(share / 'param' / 'vio_stereo.yaml')),
        DeclareLaunchArgument('calib_file', default_value=str(share / 'param' / 'calib' / 'euroc_stereo.yaml')),
        DeclareLaunchArgument('imu_topic', default_value='/imu0'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/schurvins-results'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='svo_ros', executable='svo_node', name='svo', output='screen', parameters=[
            LaunchConfiguration('params'), {
                'pipeline': LaunchConfiguration('pipeline'), 'calib_file': LaunchConfiguration('calib_file'),
                'imu_topic': LaunchConfiguration('imu_topic'), 'output_dir': LaunchConfiguration('output_dir'),
                'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])])
