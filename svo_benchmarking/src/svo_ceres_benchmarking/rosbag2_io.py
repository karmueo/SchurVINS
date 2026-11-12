"""Read ROS2 MCAP and SQLite3 without ROS1 or vendor message packages."""
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def stamp_seconds(stamp):
    return stamp_ns(stamp) * 1e-9


def open_reader(uri):
    path = Path(uri)
    if path.suffix == '.bag':
        raise ValueError('ROS1 .bag is unsupported; use ROS2 MCAP or SQLite3')
    storage = 'mcap' if path.suffix == '.mcap' else 'sqlite3' if path.suffix == '.db3' else ''
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def read_messages(uri, topics=None):
    reader = open_reader(uri)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if isinstance(topics, str):
        topics = [topics]
    if topics:
        missing = set(topics) - types.keys()
        if missing:
            raise ValueError(f'Topics not found in bag: {sorted(missing)}')
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in classes:
            classes[topic] = get_message(types[topic])
        yield topic, deserialize_message(data, classes[topic]), timestamp
