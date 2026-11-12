"""ROS2 message export formats used by the offline benchmark (id + seconds)."""
from pathlib import Path
from .rosbag2_io import read_messages, stamp_seconds


def pose_values(message):
    """Accept PoseStamped, PoseWithCovarianceStamped, Odometry, PointStamped or TransformStamped."""
    if hasattr(message, 'transform'):
        p, q = message.transform.translation, message.transform.rotation
    elif hasattr(message, 'pose'):
        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        p, q = pose.position, pose.orientation
    elif hasattr(message, 'point'):
        p, q = message.point, None
    else:
        raise TypeError(f'Unsupported pose message: {type(message).__name__}')
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w) if q is not None else (p.x, p.y, p.z, 0., 0., 0., 1.)


def export_pose(bag, topic, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with output.open('w') as stream:
        stream.write('# id timestamp tx ty tz qx qy qz qw\n')
        for _, message, _ in read_messages(bag, topics=topic):
            values = (stamp_seconds(message.header.stamp), *pose_values(message))
            stream.write(str(count) + ' ' + ' '.join(f'{v:.12f}' for v in values) + '\n')
            count += 1
    return count


def export_imu(bag, topic, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with output.open('w') as stream:
        stream.write('# id timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z\n')
        for _, message, _ in read_messages(bag, topics=topic):
            w, a = message.angular_velocity, message.linear_acceleration
            values = (stamp_seconds(message.header.stamp), w.x, w.y, w.z, a.x, a.y, a.z)
            stream.write(str(count) + ' ' + ' '.join(f'{v:.12f}' for v in values) + '\n')
            count += 1
    return count


def export_images(bag, topic, output, camera_id):
    import cv2
    from cv_bridge import CvBridge
    output = Path(output)
    image_dir = output.parent / 'img'
    image_dir.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()
    count = 0
    with output.open('w') as stream:
        stream.write('# id timestamp image_name\n')
        for _, message, _ in read_messages(bag, topics=topic):
            image = (bridge.compressed_imgmsg_to_cv2(message, 'mono8') if hasattr(message, 'format')
                     else bridge.imgmsg_to_cv2(message, 'mono8'))
            name = f'image_{camera_id}_{count}.png'
            if not cv2.imwrite(str(image_dir / name), image):
                raise OSError(f'Cannot write {image_dir / name}')
            stream.write(f'{count} {stamp_seconds(message.header.stamp):.12f} img/{name}\n')
            count += 1
    return count
