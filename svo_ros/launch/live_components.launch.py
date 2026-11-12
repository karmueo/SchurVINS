"""Generic hardware interface example; supply standard ROS2 image and IMU topics."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = Path(get_package_share_directory('svo_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('calib_file', description='SVO camera/IMU calibration YAML'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/schurvins-results'),
        ComposableNodeContainer(name='svo_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen', composable_node_descriptions=[
                ComposableNode(package='svo_ros', plugin='svo_ros::SvoNodeBase', name='svo', parameters=[
                    str(share/'param/vio_stereo.yaml'), {'calib_file':LaunchConfiguration('calib_file'),
                    'output_dir':LaunchConfiguration('output_dir'), 'use_sim_time':False}])])])
