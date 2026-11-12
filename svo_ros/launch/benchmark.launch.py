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
        DeclareLaunchArgument('dataset_directory'), DeclareLaunchArgument('calib_file'),
        DeclareLaunchArgument('dataset_is_stereo', default_value='true'),
        DeclareLaunchArgument('dataset_is_array', default_value='false'),
        DeclareLaunchArgument('dataset_is_kitti', default_value='false'),
        DeclareLaunchArgument('dataset_is_blender', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/schurvins-results'),
        Node(package='svo_ros', executable='svo_benchmark', name='svo', output='screen', parameters=[
            str(share/'param/vio_stereo.yaml'), {
                'dataset_directory':LaunchConfiguration('dataset_directory'), 'calib_file':LaunchConfiguration('calib_file'),
                'output_dir':LaunchConfiguration('output_dir'),
                **{key:ParameterValue(LaunchConfiguration(key), value_type=bool) for key in (
                    'dataset_is_stereo','dataset_is_array','dataset_is_kitti','dataset_is_blender')}}])])
