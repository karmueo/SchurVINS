"""Run independent EGO adapter and the same SVO component implementation."""
from pathlib import Path
import runpy
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def setup(context):
    share = Path(get_package_share_directory('svo_ros'))
    output = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    generated = Path(tempfile.mkdtemp(prefix='ego_calibration_', dir=output)) / 'calibration.yaml'
    source = LaunchConfiguration('source_calibration').perform(context)
    runpy.run_path(str(share / 'scripts' / 'ego_calibration.py'))['convert'](source, generated)
    components = [
        ComposableNode(package='svo_ros', plugin='svo_ros::EgoInput', name='ego_input',
                       parameters=[{'source_calibration': source, 'use_sim_time': True}]),
        ComposableNode(package='svo_ros', plugin='svo_ros::SvoNodeBase', name='svo',
                       parameters=[str(share / 'param' / 'vio_stereo.yaml'), {
                           'calib_file': str(generated), 'output_dir': str(output),
                           'pipeline': 'stereo', 'use_sim_time': True,
                           'cam0_topic': '/ego/cam0/image_raw',
                           'cam1_topic': '/ego/cam1/image_raw', 'imu_topic': '/ego/imu',
                           'image_queue_capacity': 64, 'imu_queue_capacity': 10000}]),
    ]
    actions = [ComposableNodeContainer(name='svo_container', namespace='', package='rclcpp_components',
               executable='component_container_mt', composable_node_descriptions=components, output='screen')]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz_config_vio.rviz')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_calibration', default_value='/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/schurvins-results'),
        DeclareLaunchArgument('rviz', default_value='false'), OpaqueFunction(function=setup)])
