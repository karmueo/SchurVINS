"""Native storage fixtures: run only in a sourced Jazzy install, no ROS daemon needed."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from svo_ceres_benchmarking.rosbag2_io import open_reader, read_messages, restamp, stamp_ns
from svo_ceres_benchmarking.data_export import export_pose, export_imu, export_images, pose_values


class RosbagToolsTest(unittest.TestCase):
    def make_bag(self, folder, storage):
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id=storage),
                    rosbag2_py.ConverterOptions('', ''))
        topics = [('pose', 'geometry_msgs/msg/PoseStamped'),
                  ('imu', 'sensor_msgs/msg/Imu'), ('image', 'sensor_msgs/msg/Image')]
        for i, (topic, message_type) in enumerate(topics):
            writer.create_topic(rosbag2_py.TopicMetadata(id=i, name='/' + topic,
                                type=message_type, serialization_format='cdr'))
        pose, imu, image = PoseStamped(), Imu(), Image()
        pose.pose.position.x, pose.pose.orientation.w = 2., 1.
        imu.angular_velocity.x, imu.linear_acceleration.z = 0.3, 9.8
        image.height, image.width, image.encoding, image.step = 2, 3, 'mono8', 3
        image.data = [0, 1, 2, 3, 4, 5]
        for message, (topic, _) in zip((pose, imu, image), topics):
            message.header.stamp.sec, message.header.stamp.nanosec = 10, 1234
            writer.write('/' + topic, serialize_message(message), 99_000_000_000)
        del writer

    def test_native_read_write_and_exports(self):
        import cv2
        for storage in ('mcap', 'sqlite3'):
            with self.subTest(storage=storage), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.make_bag(root / 'source', storage)
                self.assertEqual(len(list(read_messages(root / 'source'))), 3)
                self.assertEqual(restamp(root / 'source', root / 'copy', storage), 3)
                copied = list(read_messages(root / 'copy'))
                self.assertTrue(all(time == stamp_ns(msg.header.stamp) for _, msg, time in copied))
                self.assertEqual(export_pose(root / 'copy', '/pose', root / 'poses.txt'), 1)
                self.assertEqual(np.loadtxt(root / 'poses.txt')[2], 2.)
                self.assertEqual(export_imu(root / 'copy', '/imu', root / 'imu.txt'), 1)
                self.assertEqual(np.loadtxt(root / 'imu.txt')[2], 0.3)
                self.assertEqual(export_images(root / 'copy', '/image', root / 'images.txt', 0), 1)
                image = cv2.imread(str(root / 'img/image_0_0.png'), cv2.IMREAD_GRAYSCALE)
                np.testing.assert_array_equal(image, np.arange(6, dtype=np.uint8).reshape(2, 3))
                with self.assertRaises(ValueError):
                    list(read_messages(root / 'copy', '/missing'))
                with self.assertRaises(FileExistsError):
                    restamp(root / 'source', root / 'copy', storage)

    def test_original_pose_message_types(self):
        for message in (PoseStamped(), PoseWithCovarianceStamped(), Odometry(), PointStamped(), TransformStamped()):
            with self.subTest(message=type(message).__name__):
                values = pose_values(message)
                self.assertEqual(len(values), 7)
        with self.assertRaises(ValueError):
            open_reader('old.bag')


if __name__ == '__main__':
    unittest.main()
