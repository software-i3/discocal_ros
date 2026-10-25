# One camera's calibration inside a CalibrateCamera result.
#
# A mono goal returns exactly one of these, mirroring the action's scalar
# fields. A both-mode goal returns one per camera, ordered by camera name.

# Camera key in the rig config (e.g. "left").
string camera

# The camera's full verdict: the mono solve produced usable intrinsics, the
# cross-camera check passed (or was explicitly allowed not to run), and the
# camera_info was written.
bool   success
string message

# Intrinsics of this camera. The distortion model is purely radial:
# k = 1 + d1*r^2 + d2*r^4, with no tangential term. d3/d4 are zero unless the
# board config raises n_d above 2.
#
# The intrinsics and their sigmas below are zero unless `success` is true. A
# failed solve or failed check still measured numbers, but they are the numbers
# the check just rejected; they go in the log and the run's YAML, not in the
# action result, so a consumer that ignores `success` cannot install them. The
# diagnostic fields (reprojection error, frame counts, check verdict, fit time)
# are reported even on failure, because tuning a threshold needs them.
float64 fx
float64 fy
float64 cx
float64 cy
float64 skew
float64 d1
float64 d2
float64 d3
float64 d4

# 1-sigma uncertainties, from the calibrator's Jacobian (printed as 2sigma).
float64 s_fx
float64 s_fy
float64 s_cx
float64 s_cy
float64 s_skew
float64 s_d1
float64 s_d2
float64 s_d3
float64 s_d4

# True when a joint refinement replaced this camera's intrinsics but the
# uncertainties above are still the mono solve's: StereoCalibrator estimates no
# covariance for the joint stage.
bool uncertainties_from_mono_stage

float64 reprojection_error
uint32  n_frames              # accepted frames that went into this camera's solve
uint32  n_frames_heldout      # accepted frames reserved for the cross-camera check
string  camera_info_path      # written camera_info YAML; "" if not written

# Held-out cross-camera consistency check for this camera.
bool    check_ran
bool    check_passed
uint32  check_pairs
float64 check_rms_px
float64 check_max_rms_px
float64 check_worst_pair_rms_px

# Time spent in this camera's solve (mono, plus the joint stage's share is not
# attributed here; see CalibrateCameraResult.joint_fit_seconds).
float64 fit_seconds

# Board-pose source for this camera ("image" or "external"). In external mode
# the board pose is measured from point_pose_topic instead of estimated from
# the image, and the fields below describe the label/symmetry matching.
string  pose_source
uint32  frames_matched            # frames that entered the external-pose fit
uint32  frames_ambiguous          # rejected: symmetry margin below the limit
uint32  frames_rejected_external  # rejected for any external-pose reason
float64 mean_rigidity_rms_m       # mean Kabsch residual of the measured dots
