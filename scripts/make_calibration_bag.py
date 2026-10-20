#!/usr/bin/env python
"""Build a ROS 1 bag from a discocal calibration image set.

The dataset is a flat directory of ``<index>_left.ppm`` /
``<index>_right.ppm`` pairs (the same layout ``calibration_set_big`` uses).
Each pair becomes two ``sensor_msgs/CompressedImage`` messages -- JPEG-encoded
once at the requested quality -- with a *shared* stamp, so the node can pair
them across cameras and build a common holdout.

    rosrun discocal_ros make_calibration_bag.py \
        --dir /data/calibration_set_big \
        --out /output/calibration_set_big.bag \
        --repeats 2

``--repeats`` writes the whole set again with distinct stamps. This is how a
short dataset feeds a budget that is larger than the number of unique poses:
replaying the *same* bag would reuse stamps and (with the diversity gate on)
the same poses, so the extra frames would be rejected. Passes are spaced by
one dataset span plus a 10 s gap, so playing the bag at rate 1 continues
straight into the next pass instead of stalling on a multi-hour empty gap;
every stamp stays unique no matter how large the dataset is.

Python 2.7 compatible: Melodic's ``python`` is 2.7, and this script runs
inside the calibration container.
"""
from __future__ import print_function

import argparse
import hashlib
import os
import re
import sys
import time

import cv2
import rosbag
import rospy
from sensor_msgs.msg import CompressedImage

DEFAULT_LEFT_TOPIC = "/ikan/explore3d/left/compressed"
DEFAULT_RIGHT_TOPIC = "/ikan/explore3d/right/compressed"
DEFAULT_FRAME_ID = "explore3d"
STAMP_BASE = 1000.0
# A pass is one dataset span long; the gap only has to make the next pass's
# first stamp strictly later than this pass's last one.
STAMP_PASS_GAP = 10.0

_PAIR = re.compile(r"^([0-9]+)_(left|right)\.ppm$")


def find_pairs(directory):
    """Return (pairs, left_only, right_only).

    A pair is an index present as both a left and a right image. The shipped
    dataset is missing an index on both sides (000024); that shows up in
    neither list and is skipped silently. An index present on only one side is
    reported, because it usually means a truncated copy.
    """
    left = {}
    right = {}
    for name in os.listdir(directory):
        match = _PAIR.match(name)
        if match is None:
            continue
        index = int(match.group(1))
        target = left if match.group(2) == "left" else right
        target[index] = os.path.join(directory, name)

    indices = sorted(set(left) & set(right))
    pairs = [(index, left[index], right[index]) for index in indices]
    left_only = sorted(set(left) - set(right))
    right_only = sorted(set(right) - set(left))
    return pairs, left_only, right_only


def encode_jpeg(path, quality):
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read image: %s" % path)
    ok, buffer = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("JPEG encoding failed: %s" % path)
    # tostring() is the Python 2 spelling and still exists in Python 3 numpy
    # (deprecated), which keeps this script portable enough to debug on the host.
    return buffer.tostring(), image.shape[1], image.shape[0]


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def make_message(topic, stamp, frame_id, data):
    message = CompressedImage()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.format = "jpeg"
    message.data = data
    return topic, message


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", required=True,
                        help="directory containing <index>_left.ppm/<index>_right.ppm")
    parser.add_argument("--out", required=True, help="output .bag path")
    parser.add_argument("--repeats", type=int, default=1,
                        help="number of complete passes to write, with distinct "
                             "stamps per pass (default: 1)")
    parser.add_argument("--quality", type=int, default=95,
                        help="JPEG quality, 0-100 (default: 95)")
    parser.add_argument("--left-topic", default=DEFAULT_LEFT_TOPIC)
    parser.add_argument("--right-topic", default=DEFAULT_RIGHT_TOPIC)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    args = parser.parse_args()

    if args.repeats < 1:
        print("FAIL: --repeats must be at least 1")
        return 2
    if not (0 <= args.quality <= 100):
        print("FAIL: --quality must be in 0..100")
        return 2
    if not os.path.isdir(args.dir):
        print("FAIL: not a directory: %s" % args.dir)
        return 2

    pairs, left_only, right_only = find_pairs(args.dir)
    if not pairs:
        print("FAIL: no complete left/right pairs under %s" % args.dir)
        return 2
    if left_only:
        print("WARNING: %d left image(s) have no right partner, e.g. indices %s"
              % (len(left_only), left_only[:10]))
    if right_only:
        print("WARNING: %d right image(s) have no left partner, e.g. indices %s"
              % (len(right_only), right_only[:10]))

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    for stale in (args.out, args.out + ".active"):
        if os.path.exists(stale):
            os.remove(stale)
    # rosbag writes to <out>.active and renames on close, so an interrupted run
    # never leaves a file at --out that looks complete.
    started = time.time()
    print("encoding %d pair(s) from %s" % (len(pairs), args.dir))

    encoded = []
    width = height = None
    for position, (index, left_path, right_path) in enumerate(pairs):
        left_data, left_w, left_h = encode_jpeg(left_path, args.quality)
        right_data, right_w, right_h = encode_jpeg(right_path, args.quality)
        if (left_w, left_h) != (right_w, right_h):
            print("FAIL: image size mismatch at index %d: %dx%d vs %dx%d"
                  % (index, left_w, left_h, right_w, right_h))
            return 2
        if width is None:
            width, height = left_w, left_h
        elif (left_w, left_h) != (width, height):
            print("FAIL: image size changed at index %d: %dx%d, expected %dx%d"
                  % (index, left_w, left_h, width, height))
            return 2
        encoded.append((index, left_data, right_data))
        if (position + 1) % 50 == 0 or position + 1 == len(pairs):
            print("  encoded %d/%d" % (position + 1, len(pairs)))

    first_index = encoded[0][0]
    last_index = encoded[-1][0]
    pass_spacing = (last_index - first_index) + STAMP_PASS_GAP
    print("pass spacing %.1f s" % pass_spacing)

    messages = 0
    first_stamp = None
    last_stamp = None
    with rosbag.Bag(args.out, "w") as bag:
        for repeat in range(args.repeats):
            for index, left_data, right_data in encoded:
                seconds = (STAMP_BASE + repeat * pass_spacing
                           + (index - first_index))
                stamp = rospy.Time.from_sec(seconds)
                if first_stamp is None or stamp < first_stamp:
                    first_stamp = stamp
                if last_stamp is None or stamp > last_stamp:
                    last_stamp = stamp
                for topic, message in (
                        make_message(args.left_topic, stamp, args.frame_id, left_data),
                        make_message(args.right_topic, stamp, args.frame_id, right_data)):
                    bag.write(topic, message, t=stamp)
                    messages += 1

    size = os.path.getsize(args.out)
    duration = (last_stamp - first_stamp).to_sec() if first_stamp is not None else 0.0
    print("")
    print("PAIRS       %d" % len(pairs))
    print("MESSAGES    %d (%d pass(es) x %d pair(s) x 2 cameras)"
          % (messages, args.repeats, len(pairs)))
    print("STAMPS      %.3f .. %.3f (span %.1f s)"
          % (first_stamp.to_sec(), last_stamp.to_sec(), duration))
    print("IMAGE       %dx%d, JPEG quality %d, frame_id %s"
          % (width, height, args.quality, args.frame_id))
    print("TOPICS      %s, %s" % (args.left_topic, args.right_topic))
    print("BAG         %s (%d bytes)" % (args.out, size))
    print("SHA256      %s" % sha256_file(args.out))
    print("CREATED     %.1f s" % (time.time() - started))
    return 0


if __name__ == "__main__":
    sys.exit(main())
