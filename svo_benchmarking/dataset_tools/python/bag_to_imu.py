#!/usr/bin/env python3

from svo_ceres_benchmarking.rosbag2_io import read_messages, stamp_seconds, stamp_ns
import argparse

def extract(bagfile, imu_topic, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z\n')
    bag_uri = bagfile
    for (topic, msg, ts) in read_messages(bag_uri, topics=str(imu_topic)):
        f.write('%d %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                (n,
                 stamp_seconds(msg.header.stamp),
                 msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        n += 1
    print(('wrote ' + str(n) + ' imu messages to the file: ' + out_filename))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    args = parser.parse_args()

    extract(args.bag, args.topic, 'imu.txt')
