#!/usr/bin/env python3
# Original utility: www.asl.ethz.ch, IJRR MAV datasets. ROS2 port.
"""Copy a ROS2 bag using message header timestamps (MCAP or SQLite3)."""
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from svo_ceres_benchmarking.rosbag2_io import open_reader, stamp_ns


def restamp(source, destination, storage='mcap'):
    reader = open_reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=destination, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = reader.get_all_topics_and_types()
    classes = {}
    for topic in types:
        writer.create_topic(topic)
        classes[topic.name] = get_message(topic.type)
    count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        message = deserialize_message(data, classes[topic])
        if hasattr(message, 'header'):
            timestamp = stamp_ns(message.header.stamp)
        writer.write(topic, data, timestamp)
        count += 1
    del writer
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-i', '--ifile', required=True)
    parser.add_argument('-o', '--ofile', required=True)
    parser.add_argument('--storage', choices=['mcap', 'sqlite3'], default='mcap')
    args = parser.parse_args()
    print((restamp(args.ifile, args.ofile, args.storage)))
