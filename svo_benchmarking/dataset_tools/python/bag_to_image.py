#!/usr/bin/env python3

from svo_ceres_benchmarking.rosbag2_io import read_messages, stamp_seconds, stamp_ns
import argparse
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def extract(bagfile, pose_topic, out_filename, cam_id):
    n = 0
    f = open(out_filename, 'w')
    f.write('# id timestamp image_name\n')
    cv_bridge = CvBridge()
    extract_every_nth_image = 1
    #max_imgs = 400

    bag_uri = bagfile
    for (topic, msg, ts) in read_messages(bag_uri, topics=str(pose_topic)):
        if np.mod(n, extract_every_nth_image) == 0:
            try:
                img = cv_bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') if hasattr(msg, 'format') else cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                raise RuntimeError(str(e)) from e

            ts = stamp_seconds(msg.header.stamp)
            image_name = 'image_'+str(cam_id)+'_'+str(n)+'.png'
            f.write('%d %.12f img/%s \n' % (n, ts, image_name))
            from pathlib import Path
            image_dir = Path(out_filename).resolve().parent / 'img'
            image_dir.mkdir(exist_ok=True)
            if not cv2.imwrite(str(image_dir / image_name), img):
                raise RuntimeError('Failed to write image')
        n += 1
        #if n > max_imgs:
        #    break

    print(('wrote ' + str(n) + ' images messages to the file: ' + out_filename))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts Images messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('cam_id', help='Camera-Id')
    args = parser.parse_args()
    print(('Extract images from bag '+args.bag+' in topic ' + args.topic))
    extract(args.bag, args.topic, 'images.txt', args.cam_id)
