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


def restamp(source, destination, storage='mcap'):
    """Copy serialized ROS2 messages, substituting available standard header stamps.

    Unknown custom messages remain serialized and keep their bag timestamps;
    installing their message package is unnecessary for a lossless copy.
    """
    if storage not in ('mcap', 'sqlite3'):
        raise ValueError('Only mcap and sqlite3 are supported')
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(f'Refusing to overwrite bag output: {destination}')
    if destination.suffix == '.bag':
        raise ValueError('ROS1 .bag output is unsupported')
    destination.parent.mkdir(parents=True, exist_ok=True)
    reader = open_reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    classes = {}
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)
        try:
            classes[topic.name] = get_message(topic.type)
        except (ImportError, AttributeError, ValueError, RuntimeError):
            classes[topic.name] = None
    count = 0
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if classes[topic] is not None:
                message = deserialize_message(data, classes[topic])
                if hasattr(message, 'header'):
                    timestamp = stamp_ns(message.header.stamp)
            writer.write(topic, data, timestamp)
            count += 1
    finally:
        # SequentialWriter flushes and closes its storage in its destructor.
        del writer
    return count
