#!/usr/bin/env python
"""Publish a directory of images as a stereo feed, for testing discocal_ros.

Reads `<dir>/<index>_left.<ext>` and `<dir>/<index>_right.<ext>` (the layout of
calibration_set_big) and republishes them as sensor_msgs/CompressedImage on the
two topics from the rig config.

Both cameras of one index are published with the SAME header stamp, because that
is what discocal_ros pairs on -- an exact match on `header.stamp`. If you change
that here without changing the node, every held-out frame becomes unpaired and
the cross-camera check reports "no usable held-out pairs".

Usage:
    rosrun discocal_ros publish_image_dir.py --dir calibration_set_big \
        --left-topic /ikan/explore3d/left/compressed \
        --right-topic /ikan/explore3d/right/compressed

    # One pass through the directory, then exit (useful for a scripted run):
    rosrun discocal_ros publish_image_dir.py --dir calibration_set_big --once

    # Slow it down so a human can watch the node's feedback:
    rosrun discocal_ros publish_image_dir.py --dir calibration_set_big --rate 5
"""

from __future__ import print_function

import argparse
import glob
import os
import re
import sys
import time

import cv2
import rospy
from sensor_msgs.msg import CompressedImage


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", required=True,
                        help="directory of <index>_left.<ext> / <index>_right.<ext> pairs")
    parser.add_argument("--left-topic", default="/ikan/explore3d/left/compressed")
    parser.add_argument("--right-topic", default="/ikan/explore3d/right/compressed")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="publish rate in Hz; 0 means as fast as possible")
    parser.add_argument("--once", action="store_true",
                        help="exit after one pass instead of looping")
    parser.add_argument("--jpeg-quality", type=int, default=95,
                        help="JPEG quality for the compressed images")
    # A zero stamp would make every frame pair with every other one, so the
    # node drops those frames outright. Start the clock somewhere clearly
    # non-zero.
    parser.add_argument("--start-sec", type=int, default=1000)
    return parser.parse_args()


def index_of(path):
    """The leading integer of a filename, or None."""
    match = re.match(r"^(\d+)", os.path.basename(path))
    return int(match.group(1)) if match else None


def collect_pairs(directory):
    """[(index, left_path, right_path)], sorted by index, skipping half-pairs."""
    lefts = {}
    rights = {}
    for path in sorted(glob.glob(os.path.join(directory, "*_left.*"))):
        index = index_of(path)
        if index is not None:
            lefts[index] = path
    for path in sorted(glob.glob(os.path.join(directory, "*_right.*"))):
        index = index_of(path)
        if index is not None:
            rights[index] = path

    pairs = []
    for index in sorted(lefts):
        if index in rights:
            pairs.append((index, lefts[index], rights[index]))
        else:
            print("skipping %d: no right image" % index, file=sys.stderr)
    for index in sorted(rights):
        if index not in lefts:
            print("skipping %d: no left image" % index, file=sys.stderr)
    return pairs


def make_message(path, stamp, quality):
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise IOError("cannot read %s" % path)
    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise IOError("cannot encode %s" % path)

    message = CompressedImage()
    message.header.stamp = stamp
    message.header.frame_id = "explore3d"
    message.format = "jpeg"
    message.data = buffer.tostring()
    return message


def main():
    args = parse_args()
    if not os.path.isdir(args.dir):
        print("not a directory: %s" % args.dir, file=sys.stderr)
        return 1

    pairs = collect_pairs(args.dir)
    if not pairs:
        print("no <index>_left.<ext> / <index>_right.<ext> pairs in %s" % args.dir,
              file=sys.stderr)
        return 1

    rospy.init_node("publish_image_dir", anonymous=True)
    left_pub = rospy.Publisher(args.left_topic, CompressedImage, queue_size=4)
    right_pub = rospy.Publisher(args.right_topic, CompressedImage, queue_size=4)

    # Subscribers are discovered asynchronously, and a frame published before any
    # subscriber exists is dropped by the graph -- which would silently shorten
    # the first pass. Wait for both.
    while not rospy.is_shutdown() and (left_pub.get_num_connections() == 0 or
                                       right_pub.get_num_connections() == 0):
        rospy.sleep(0.1)

    period = 1.0 / args.rate if args.rate > 0 else 0.0
    print("publishing %d pair(s) from %s" % (len(pairs), args.dir))

    passes = 0
    while not rospy.is_shutdown():
        passes += 1
        for index, left_path, right_path in pairs:
            if rospy.is_shutdown():
                return 0
            # One stamp for both cameras: the node pairs on exact equality.
            stamp = rospy.Time(args.start_sec + index)
            left_pub.publish(make_message(left_path, stamp, args.jpeg_quality))
            right_pub.publish(make_message(right_path, stamp, args.jpeg_quality))
            if period > 0:
                rospy.sleep(period)
        print("pass %d done (%d pair(s))" % (passes, len(pairs)))
        if args.once:
            break

    # Give the last messages a chance to be delivered before the node exits and
    # the publisher sockets close.
    rospy.sleep(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
