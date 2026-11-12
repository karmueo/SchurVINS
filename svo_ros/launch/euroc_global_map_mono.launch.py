from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('svo_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/schurvins-results'),
        DeclareLaunchArgument('vocabulary_dir', description='Directory containing the locked official vocabulary'),
        Node(package='svo_ros', executable='svo_node', name='svo', output='screen', parameters=[
            str(share/'param/vio_mono.yaml'), str(share/'param/global_map.yaml'), {
                'pipeline':'mono', 'use_global_map':True, 'runlc':True,
                'voc_path':LaunchConfiguration('vocabulary_dir'),
                'calib_file':str(share/'param/calib/euroc_mono.yaml'), 'imu_topic':'/imu0',
                'use_sim_time':True, 'output_dir':LaunchConfiguration('output_dir')}])])
