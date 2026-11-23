"""Run EGO from live ROS topics or process a complete MCAP in one command."""
from pathlib import Path
import runpy
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def offline_finished(event, context):
    if event.returncode:
        raise RuntimeError(f'EGO MCAP processing failed with exit code {event.returncode}; inspect the output summary')
    return [EmitEvent(event=Shutdown(reason='EGO MCAP processing completed'))]


def setup(context):
    share = Path(get_package_share_directory('svo_ros'))
    helpers = runpy.run_path(str(share / 'scripts' / 'ego_launch_support.py'))
    value = lambda name: LaunchConfiguration(name).perform(context)
    save_trajectory = helpers['boolean'](value('save_trajectory'))
    realtime = helpers['boolean'](value('realtime'))
    rviz = helpers['boolean'](value('rviz'))
    inbag = value('inbag')
    if inbag:
        bag = Path(inbag).expanduser().resolve()
        if bag.suffix != '.mcap' or not bag.is_file():
            raise ValueError(f'inbag must be an existing .mcap file: {bag}')
        inbag = str(bag)
    bag_name = Path(inbag).stem if inbag else 'live'
    cameras = Path(value('cameras')).expanduser().resolve()
    if not cameras.is_file():
        raise FileNotFoundError(f'Camera calibration does not exist: {cameras}')
    output = helpers['output_directory'](value('output_dir'), bag_name)
    helpers['check_outputs'](output, bag_name, bool(inbag), save_trajectory)
    generated = Path(tempfile.mkdtemp(prefix='ego_calibration_', dir=output)) / 'calibration.yaml'
    runpy.run_path(str(share / 'scripts' / 'ego_calibration.py'))['convert'](cameras, generated)
    params = Path(value('params')).expanduser().resolve()
    if not params.is_file():
        raise FileNotFoundError(f'Parameter file does not exist: {params}')
    parameters = [str(params), {
        'calib_file': str(generated), 'output_dir': str(output),
        'pipeline': 'stereo', 'use_sim_time': True,
        'save_trajectory': save_trajectory,
        'trajectory_file': str(output / f'schurvins_{bag_name}_trajectory.txt'),
        'cam0_topic': '/ego/cam0/image_raw',
        'cam1_topic': '/ego/cam1/image_raw', 'imu_topic': '/ego/imu',
        'image_queue_capacity': 64, 'imu_queue_capacity': 10000}]
    if inbag:
        parameters.append({'inbag': inbag, 'cameras': str(cameras), 'realtime': realtime,
                           'trace_statistics': False})
        runner = Node(package='svo_ros', executable='ego_bag_node', name='svo',
                      parameters=parameters, output='screen')
        actions = [RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=offline_finished)), runner]
    else:
        components = [
            ComposableNode(package='svo_ros', plugin='svo_ros::EgoInput', name='ego_input',
                           parameters=[{'cameras': str(cameras), 'use_sim_time': True}]),
            ComposableNode(package='svo_ros', plugin='svo_ros::SvoNodeBase', name='svo',
                           parameters=parameters),
        ]
        actions = [ComposableNodeContainer(name='svo_container', namespace='', package='rclcpp_components',
                   executable='component_container_mt', composable_node_descriptions=components, output='screen')]
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz_config_vio.rviz')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(Path(get_package_share_directory('svo_ros')) / 'param' / 'vio_stereo.yaml')),
        DeclareLaunchArgument('inbag', default_value=''),
        DeclareLaunchArgument('save_trajectory', default_value='false'),
        DeclareLaunchArgument('realtime', default_value='false'),
        DeclareLaunchArgument('cameras', default_value='/data/scl_data/slam/stereo_live_bags/EGO/800hz/calib.yaml'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'), OpaqueFunction(function=setup)])
