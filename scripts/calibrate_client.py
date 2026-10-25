#!/usr/bin/env python
"""Send one CalibrateCamera goal and print everything that comes back.

The node calibrates on its own once collection reaches the budget, so a client
only has to name a camera and wait -- there is no second "solve" call. This is
also the reference for wiring the action into something else: send a goal with a
feedback callback, wait for the result, read the fields.

    rosrun discocal_ros calibrate_client.py --camera left

Exit status is 0 only when the goal succeeded and every camera in `cameras[]`
succeeded -- for a both-mode goal that means both cameras calibrated and each
one's cross-camera check passed (or was explicitly waved through with
--allow-unchecked). It is usable directly from a script:

    rosrun discocal_ros calibrate_client.py --camera left && echo ok
    rosrun discocal_ros calibrate_client.py --camera both && echo rig ok
"""
from __future__ import print_function

import argparse
import os
import sys

import actionlib
import rospy

from discocal_ros.msg import CalibrateCameraAction, CalibrateCameraGoal

DEFAULT_ACTION = "/discocal_ros/calibrate_camera"


def make_stdout_unbuffered():
    # Python 2.7's print has no flush kwarg, so feedback lines would otherwise sit
    # in the buffer until the goal finished -- which is exactly when they are
    # least useful. Not every stdout has a fileno (it can be a StringIO under a
    # test harness or an IDE), so this is best-effort.
    try:
        sys.stdout = os.fdopen(sys.stdout.fileno(), "w", 0)
    except (AttributeError, IOError, ValueError, OSError):
        pass


def feedback_cb(fb):
    # fb.state is the node's phase (COLLECTING, then SOLVING/CHECKING); the
    # counters are only meaningful during COLLECTING.
    #
    # Mono goals send one cameras[] entry mirroring the scalar fields, so the
    # legacy one-line format stays exactly as it was. Both-mode goals send one
    # entry per camera, and the scalars mirror only the first.
    if len(fb.cameras) > 1:
        for cam in fb.cameras:
            line = ("%-11s %-8s seen=%-4d detected=%-4d accepted=%-3d dup=%-4d "
                    "unpaired=%-3d budget=%-3d %3.0f%%"
                    % (fb.state, cam.camera, cam.frames_seen, cam.frames_detected,
                       cam.frames_accepted, cam.frames_duplicate,
                       cam.frames_unpaired, cam.budget, 100.0 * cam.progress))
            if cam.frames_pose_missing or cam.frames_pose_invalid:
                line += (" pose_missing=%d pose_invalid=%d"
                         % (cam.frames_pose_missing, cam.frames_pose_invalid))
            print(line)
        return
    line = ("%-11s seen=%-4d detected=%-4d accepted=%-3d dup=%-4d "
            "unpaired=%-3d budget=%-3d %3.0f%%"
            % (fb.state, fb.frames_seen, fb.frames_detected, fb.frames_accepted,
               fb.frames_duplicate, fb.frames_unpaired, fb.budget,
               100.0 * fb.progress))
    if fb.frames_pose_missing or fb.frames_pose_invalid:
        line += (" pose_missing=%d pose_invalid=%d"
                 % (fb.frames_pose_missing, fb.frames_pose_invalid))
    print(line)


def print_camera(c):
    print("  [%s] success            %s" % (c.camera, c.success))
    print("  [%s] message            %s" % (c.camera, c.message))
    print("  [%s] intrinsics         fx=%.6f fy=%.6f cx=%.6f cy=%.6f skew=%.6f"
          % (c.camera, c.fx, c.fy, c.cx, c.cy, c.skew))
    print("  [%s] distortion         d1=%.8f d2=%.8f d3=%.8f d4=%.8f"
          % (c.camera, c.d1, c.d2, c.d3, c.d4))
    print("  [%s] sigma              fx=%.6f fy=%.6f cx=%.6f cy=%.6f skew=%.6f"
          % (c.camera, c.s_fx, c.s_fy, c.s_cx, c.s_cy, c.s_skew))
    print("  [%s] sigma distortion   d1=%.8f d2=%.8f d3=%.8f d4=%.8f"
          % (c.camera, c.s_d1, c.s_d2, c.s_d3, c.s_d4))
    if c.uncertainties_from_mono_stage:
        print("  [%s] sigmas are from the mono stage; the joint stage estimates "
              "none" % c.camera)
    print("  [%s] reprojection_error %.6f px" % (c.camera, c.reprojection_error))
    print("  [%s] n_frames           %d (%d held out)"
          % (c.camera, c.n_frames, c.n_frames_heldout))
    print("  [%s] pose_source        %s"
          % (c.camera, c.pose_source or "image"))
    if c.pose_source == "external":
        print("  [%s] external match     matched=%d ambiguous=%d rejected=%d"
              % (c.camera, c.frames_matched, c.frames_ambiguous,
                 c.frames_rejected_external))
        print("  [%s] rigidity_rms_m     %.6f"
              % (c.camera, c.mean_rigidity_rms_m))
    print("  [%s] fit_seconds        %.3f" % (c.camera, c.fit_seconds))
    print("  [%s] camera_info_path   %s"
          % (c.camera, c.camera_info_path or "(none)"))
    print("  [%s] check              ran=%s passed=%s pairs=%d"
          % (c.camera, c.check_ran, c.check_passed, c.check_pairs))
    print("  [%s] check_rms_px       %.6f   (limit %.6f)"
          % (c.camera, c.check_rms_px, c.check_max_rms_px))
    print("  [%s] check_worst_pair   %.6f"
          % (c.camera, c.check_worst_pair_rms_px))


def print_result(r):
    print("")
    print("RESULT")
    print("  success                 %s" % r.success)
    print("  message                 %s" % r.message)
    print("  attempts                %d" % r.attempts)
    # Mono compatibility block. These fields mirror cameras[0] for a mono goal
    # and are zeroed for a both-mode goal.
    print("  intrinsics              fx=%.6f fy=%.6f cx=%.6f cy=%.6f skew=%.6f"
          % (r.fx, r.fy, r.cx, r.cy, r.skew))
    print("  distortion              d1=%.8f d2=%.8f d3=%.8f d4=%.8f"
          % (r.d1, r.d2, r.d3, r.d4))
    print("  sigma                   fx=%.6f fy=%.6f cx=%.6f cy=%.6f skew=%.6f"
          % (r.s_fx, r.s_fy, r.s_cx, r.s_cy, r.s_skew))
    print("  sigma distortion        d1=%.8f d2=%.8f d3=%.8f d4=%.8f"
          % (r.s_d1, r.s_d2, r.s_d3, r.s_d4))
    print("  reprojection_error      %.6f px" % r.reprojection_error)
    print("  n_frames                %d" % r.n_frames)
    print("  n_frames_heldout        %d" % r.n_frames_heldout)
    print("  pose_source             %s" % (r.pose_source or "image"))
    if r.pose_source == "external":
        print("  external match          matched=%d ambiguous=%d rejected=%d"
              % (r.frames_matched, r.frames_ambiguous,
                 r.frames_rejected_external))
        print("  mean_rigidity_rms_m     %.6f" % r.mean_rigidity_rms_m)
    print("  camera_info_path        %s" % (r.camera_info_path or "(none)"))
    print("  check_ran               %s" % r.check_ran)
    print("  check_passed            %s" % r.check_passed)
    print("  check_pairs             %d" % r.check_pairs)
    # The measurement and the threshold it had to beat, both, so the line is
    # readable without the config that produced it.
    print("  check_rms_px            %.6f   (limit %.6f)"
          % (r.check_rms_px, r.check_max_rms_px))
    print("  check_worst_pair_rms_px %.6f" % r.check_worst_pair_rms_px)

    print("  joint_refined           %s" % r.joint_refined)
    print("  joint_fit_seconds       %.3f" % r.joint_fit_seconds)
    print("  run_directory           %s" % (r.run_directory or "(none)"))
    print("  dwecal_path             %s" % (r.dwecal_path or "(none)"))
    print("  dwecal_sha256           %s" % (r.dwecal_sha256 or "(none)"))

    print("")
    print("CAMERAS (%d)" % len(r.cameras))
    for c in r.cameras:
        print_camera(c)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", default="left",
                        help="camera name from the rig config, or both/all to "
                             "calibrate every source: calibrate camera in one "
                             "goal (default: left)")
    parser.add_argument("--budget", type=int, default=0,
                        help="accepted frames to collect; 0 = the rig config's "
                             "frames.budget (default: 0)")
    parser.add_argument("--allow-unchecked", action="store_true",
                        help="write the result even if the cross-camera check "
                             "could not run. Does NOT overrule a check that ran "
                             "and failed.")
    parser.add_argument("--pose-source", default="", choices=["", "image", "external"],
                        help="board-pose source for this goal: image (estimate the "
                             "pose from the detections) or external (fit it to the "
                             "per-camera point_pose_topic PoseArray). Empty keeps "
                             "the rig config's solve.pose_source. A both-camera "
                             "goal applies the same source to both cameras "
                             "(default: empty)")
    parser.add_argument("--action", default=DEFAULT_ACTION,
                        help="action server name (default: %s)" % DEFAULT_ACTION)
    parser.add_argument("--timeout", type=float, default=900.0,
                        help="seconds to wait for the result (default: 900)")
    parser.add_argument("--quiet", action="store_true",
                        help="print only the result line, not per-frame feedback")
    args = parser.parse_args()

    make_stdout_unbuffered()
    rospy.init_node("calibrate_client", anonymous=True)

    client = actionlib.SimpleActionClient(args.action, CalibrateCameraAction)
    print("waiting for %s ..." % args.action)
    if not client.wait_for_server(rospy.Duration(60.0)):
        print("FAIL: no action server at %s. Is the node running?" % args.action)
        return 2

    goal = CalibrateCameraGoal()
    goal.camera = args.camera
    goal.budget = args.budget
    goal.allow_unchecked = args.allow_unchecked
    goal.pose_source = args.pose_source
    print("sending goal camera=%s budget=%d allow_unchecked=%s pose_source=%s"
          % (args.camera, args.budget, args.allow_unchecked,
             args.pose_source or "(config)"))
    client.send_goal(goal, feedback_cb=None if args.quiet else feedback_cb)

    if not client.wait_for_result(rospy.Duration(args.timeout)):
        print("FAIL: no result within %.0f s. The node is still collecting, or "
              "frames.max_duration_s is larger than --timeout." % args.timeout)
        return 3

    r = client.get_result()
    if r is None:
        # The server died or the goal was preempted: the terminal state says which.
        print("FAIL: no result (action state %d)" % client.get_state())
        return 4

    print_result(r)
    # `r.success` is the goal's overall verdict, but a both-mode goal is only
    # useful if every camera passed its own check as well. Requiring both keeps
    # this honest when a scalar-only consumer would otherwise see success.
    all_cameras_ok = len(r.cameras) > 0 and all(c.success for c in r.cameras)
    return 0 if (r.success and all_cameras_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
