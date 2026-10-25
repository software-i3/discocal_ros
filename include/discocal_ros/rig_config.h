#ifndef DISCOCAL_ROS_RIG_CONFIG_H_
#define DISCOCAL_ROS_RIG_CONFIG_H_

// Rig description: the board, the frame policy, the check policy, and the
// cameras (topics + how each camera's intrinsics are obtained). Parsed from the
// YAML passed as ~rig_config. See config/rig_example.yaml for the annotated
// schema.
//
// This header deliberately includes only the vendored library's utils.h (for
// ::Params and ::Shape) and not CCalibrator.h/CTargetDetector.h: those two do
// `using namespace std;` at global scope, and anything that includes them
// inherits it. The classes that need them include them in a .cpp.

#include <map>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "utils.h"  // ::Params, ::Shape

namespace discocal_ros {

// The calibration board. n_x dots along a row, n_y along a column; `type`
// selects the detector's and calibrator's algorithm for it.
struct BoardConfig {
    // "circle" -> conic/moment path, Calibrator::calibrate(0).
    // "square" -> corner path, Calibrator::calibrate(2).
    std::string type = "circle";
    int n_x = 7;
    int n_y = 5;
    bool asymmetric = false;
    // Metres. radius is the dot RADIUS (not diameter); distance is the
    // centre-to-centre pitch.
    double radius = 0.0145;
    double distance = 0.039;
    // Distortion order estimated: d1..d_n_d. 2 is the library's usual setting.
    int n_d = 2;
    // "" -> BGR2GRAY; "saturation" -> the S channel of HSV. Passed straight to
    // TargetDetector::preprocessing, which is where it is interpreted.
    std::string detection_mode = "";

    // The detector's inside-black/surround-white appearance gate for each
    // candidate blob, exposed as DISC_EDGE_CHECK in the vendored sources.
    //
    // It matters more than it sounds. With the gate off, the detector keeps only
    // the n_x*n_y LARGEST blobs (sorted by area, cap at CTargetDetector.cpp:665),
    // so any static dark object in the room wins a seat and evicts a real dot.
    // Measured on calibration_set_big: 48/294 (left) and 56/294 (right) frames
    // detect with the gate off, against 275/294 and 279/294 with it on, and the
    // frames that already worked produce the same calibration either way.
    //
    // That is fatal for this node rather than merely wasteful: at 5-16%
    // detection a 50-frame budget does not fill before frames.max_duration_s,
    // and because the two cameras fail independently, almost no accepted frame
    // finds a partner for the cross-camera check either.
    //
    // It defaults ON here, the opposite of the vendored library. The library
    // defaults off so that stock runs stay byte-identical to earlier
    // measurements; this node has no such history to preserve. Set false for
    // stock behaviour -- and note that a false here must stay false, which is
    // why the node writes "0" rather than leaving the variable unset.
    bool edge_check = true;
};

// Numeric knobs for the vendored detector (CTargetDetector.cpp's DISC_* tuning
// variables). The node applies these with setenv() before the first detector is
// constructed, so they take effect for every detection in the goal.
//
// A numeric value of 0 means "leave the library default alone" rather than
// "set the variable to zero": 0 is out of range for all of them, and the
// library treats an unset variable as its stock value. `probe` is
// presence-based (DISC_DEBUG_PROBE), so false must leave the variable unset
// rather than set it to "0".
struct DetectorConfig {
    double size_threshold_px2 = 0.0;     // DISC_SIZE_THRESHOLD
    double dist_threshold_px = 0.0;      // DISC_DIST_THRESHOLD
    int max_candidates = 0;              // DISC_MAX_CANDIDATES
    double edge_contrast = 0.0;          // DISC_EDGE_CONTRAST
    double edge_inner_uniformity = 0.0;  // DISC_EDGE_INNER_UNIF
    double edge_outer_uniformity = 0.0;  // DISC_EDGE_OUTER_UNIF
    bool probe = false;                  // DISC_DEBUG_PROBE (presence)
    bool edge_stats = false;             // DISC_EDGE_STATS
};

// How the node spends CPU. Every field maps to an environment variable the
// vendored sources already read, except nice/opencv/async_spinner, which are
// applied directly by the node.
struct PerformanceConfig {
    // Tier 2 uncertainty accumulation (linear in frames, ~9 s at 100 frames
    // per camera). Leave on; turning it off enables the dense path, which is
    // O(N^3) and OOMs on a full-length collection.
    bool sparse_info = true;   // DISC_SPARSE_INFO
    // Tier 1 alternative, only consulted when sparse_info is false:
    // block-diagonal M_cov accumulation. Still O(N^2) in memory.
    bool block_cov = false;    // DISC_BLOCK_COV
    // SPARSE_SCHUR for the joint refine. Ignored by the mono Calibrator, which
    // always chooses its solver from the problem shape.
    bool sparse_schur = true;  // DISC_SPARSE_SCHUR
    // Print the 1-sigma vector with 17 significant digits (DISC_UNC_DUMP) and
    // the stage timing table (DISC_TIME). Diagnostic only.
    bool unc_dump = false;
    bool time = false;
    // OpenMP threads for the detector's cal_shape_cov loop. 0 = OpenMP default.
    // The limit is applied on the detection worker thread, not in the node
    // constructor: OpenMP's nthreads-var is per-thread, so a constructor call
    // would only limit the main thread.
    int detector_threads = 0;
    // Ceres worker threads. 1 keeps the solve on one core, which matches the
    // single detector worker; raise it when a solve-only workload should use
    // the machine.
    int ceres_threads = 1;
    // cv::setNumThreads() for decode/detector/rectification. 1 avoids OpenCV's
    // own pool competing with the worker.
    int opencv_threads = 1;
    // Linux nice value for the worker thread, [-20, 19].
    int nice = 0;
    // Threads for the dedicated image callback queue's AsyncSpinner. Decoding
    // happens on these; detection stays on the one worker.
    int async_spinner_threads = 1;
    // Keep decoding (and dropping) frames while a solve runs. Off by default:
    // the solve is CPU-bound and dropping already-decoded frames is wasted
    // work; the queue would fill and evict anyway.
    bool collect_during_solve = false;
};

// Joint refinement policy for both-camera goals. Mono goals ignore this group.
struct SolveConfig {
    // Where each frame's board pose comes from.
    //
    //   "image"    -- the library's usual path: estimate a board pose per
    //                 frame from the detected dots and refine K/d and the
    //                 poses together. The detector's vector index is assumed
    //                 to match board_lattice's index.
    //   "external" -- every accepted frame carries the target dots' positions
    //                 in that camera's optical frame (a geometry_msgs/PoseArray
    //                 per camera). The board pose is then known, not estimated,
    //                 and only K/d are solved. The external points are indexed
    //                 by the dot's original label, so an index permutation is
    //                 resolved per frame from the lattice's symmetry group.
    //
    // The exposed value is the default for a goal; CalibrateCamera's
    // `pose_source` can override it per goal.
    std::string pose_source = "image";
    // Refine both intrinsic sets (and all board poses) together with the
    // extrinsic held bit-constant. Circle boards only. Not applicable to
    // "external", where the poses are already known and there are no shared
    // free board-pose parameters; it is skipped with a warning.
    bool joint_refine = true;
    // When the joint solve fails, fall back to the two mono results instead of
    // failing the goal. The fallback still evaluates and writes the artifacts,
    // with joint_refined: false in the run summary.
    bool joint_refine_fallback = true;
};

// Tuning for solve.pose_source == "external".
struct ExternalConfig {
    // --- time synchronization ---------------------------------------------
    //
    // The arm is moving continuously, so the pose sample at the image stamp is
    // what the pixel observations actually correspond to. Poses arrive on
    // their own topic and are looked up by header.stamp.
    double sync_tolerance_s = 0.02;
    // Largest spacing between the two bracketing pose samples that may be
    // interpolated. A larger gap falls back to the nearest sample (still
    // subject to sync_tolerance_s).
    double max_gap_s = 0.1;
    // Linear interpolation between bracketing samples, per dot label. False
    // selects the nearest sample.
    bool interpolate = true;

    // --- matching ----------------------------------------------------------
    //
    // The teammate's publisher indexes points by the label the dot had before
    // it was glued to the board; it does not know or care whether the board is
    // flipped. The nominal lattice's dihedral symmetries are therefore tried
    // per frame, and the external points are reordered to the detector's
    // order through the best one.
    //
    // How far the external dot cloud may be from a rigid copy of the nominal
    // lattice. This validates the pose independently of K, because the
    // rigidity residual is invariant under a relabeling of a symmetric
    // lattice. Metres.
    double max_rigidity_rms_m = 0.003;
    // Best symmetry candidate's conic reprojection RMS must be below this.
    double max_match_rms_px = 3.0;
    // If the second-best symmetry is within this of the best, the frame is
    // ambiguous. Labels normally disambiguate even a face-on symmetric board;
    // this fires when the competing candidates project onto nearly the same
    // pixels (small, distant, or edge-on board) or when detector noise is of
    // that order.
    double min_match_margin_px = 0.25;
    // A frame whose previous permutation still scores within this of the best
    // keeps the previous permutation, so a borderline frame does not flicker
    // between two labels. Set to 0 to disable temporal continuity.
    double temporal_margin_px = 0.5;
    // "skip" -> ambiguous frames do not enter the solve.
    // "use_best" -> they enter with their best-scoring permutation.
    std::string ambiguous_policy = "skip";
    // EM iterations: match with the current K/d, solve K/d with the matched
    // permutations fixed, rematch, repeat until the permutations are stable.
    int max_em_iterations = 4;
    // Initialize K from the known 3D-2D correspondences before matching. The
    // points are already in camera frame, so the pinhole model is linear in
    // (fx, skew, cx, fy, cy); no board pose or Zhang homography is involved.
    bool use_dlt_initialization = true;
    // Use the detector's per-dot covariance to weight the reprojection
    // residuals, exactly as the vendored CalibrationFunctor does.
    bool use_covariance_weights = true;

    // --- validation --------------------------------------------------------
    // Fallback check on the PoseArray's header.frame_id for cameras without
    // their own cameras.<name>.point_pose_frame_id. The comparison is exact.
    // Empty disables the check for cameras that also leave the per-camera name
    // empty.
    std::string expected_frame_id;
};

// How many frames to collect and which of them survive the pose-diversity gate.
struct FrameConfig {
    // Accepted frames to collect before solving. This is a CEILING, not a
    // requirement: when the collection window (`max_duration_s`) or the feed
    // ends first, the solve runs on whatever was accepted. A shorter collection
    // constrains the fit less and moves the reported uncertainties, but only a
    // count below the calibrator's own floor (CalibrateSession::min_fit_frames)
    // is fatal.
    int budget = 50;
    // How long to keep collecting before solving with what has arrived.
    //
    // This ends collection; it does not fail the goal. How much time filling
    // `budget` takes is set by the detector, not the feed: one worker thread
    // sustains roughly 1.4-1.8 frames/s, and only about a third of detections
    // pass the gate. A throttled test feed therefore takes minutes for 50
    // frames while a real 30 Hz rig takes seconds, so this one value cannot be
    // right for both. 120 s is a sane default for a real rig; a test feed needs
    // more, and cutting collection short yields a smaller calibration rather
    // than a failed one. The example config uses 300 for that reason.
    double max_duration_s = 120.0;
    bool diversity_gate = true;
    double diversity_threshold = 0.035;
    // Share of accepted frames held back from the solve and used only by the
    // cross-camera check, so the check is not scored on the fitted frames.
    double holdout_fraction = 0.2;
};

struct CheckConfig {
    bool enabled = true;
    // Held-out cross-camera reprojection RMS the calibration must beat.
    //
    // There is no meaningful default: the right value is a property of the rig,
    // roughly 3x the RMS its own trusted calibration scores. 4.0 is a guess
    // that at least sits above a real rig's floor rather than below it -- this
    // package's example rig floors at 1.4467 px, so the 1.0 that used to be here
    // could not be passed by any calibration at all. Tune it, and see
    // config/rig_example.yaml for how.
    double max_rms_px = 4.0;
    // Collection attempts after a failed check. 1 means report, retry once.
    int retries = 1;
};

struct CameraConfig {
    std::string name;
    std::string topic;
    // "compressed" (sensor_msgs/CompressedImage) or "raw" (sensor_msgs/Image).
    std::string transport = "compressed";
    std::string frame_id;
    // "calibrate" -> this node solves it; "file" -> taken from `intrinsics`.
    std::string source = "calibrate";
    // Only meaningful when source == "file". `has_intrinsics` distinguishes "the
    // block was present" from "it parsed to the zero Params".
    bool has_intrinsics = false;
    ::Params intrinsics;

    // geometry_msgs/PoseArray of the target dots in this camera's OPTICAL
    // frame (z forward, x right, y down -- the frame the intrinsics model
    // projects into). Array index = the dot's original label. Required for a
    // source: calibrate camera when solve.pose_source is "external"; optional
    // otherwise, and may be set so a per-goal pose_source override can use it.
    std::string point_pose_topic;
    // Optical frame the PoseArray's header.frame_id must carry for this camera.
    // Empty falls back to external.expected_frame_id. Per-camera is the useful
    // spelling on a stereo rig: two optical frames are never the same frame.
    std::string point_pose_frame_id;
};

// The fixed relative pose of `child` in `parent`:
//
//     p_parent = R * p_child + t          (t = child's origin in parent's frame)
//
// Never estimated by this node. This direction is the one discocal's own stereo
// driver writes: `relative_camera_poses.txt` says "Relative poses described in
// camera0 coordinates (p_0 = T_i^0 p_i)" and lists r_x r_y r_z, so its
// camera0 -> camera1 row pastes in here as-is with parent = camera0's directory
// and child = camera1's.
//
// It is NOT the direction of the libCalib JSON that axis_grasp consumes. That
// one is OpenCV stereoCalibrate's (R, T), which maps the other way --
// `p_1 = R * p_0 + T` -- so pasting it in needs an inversion:
//
//     R_here = R_json.transpose(),  t_here = -R_json.transpose() * T_json
//
// See rotation_from_rxryrz for the one thing the two formats do agree on.
struct ExtrinsicConfig {
    bool present = false;
    std::string parent;
    std::string child;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct OutputConfig {
    std::string directory = "/tmp/discocal_ros";
};

// .dwecal generation. The format carries no baseline and no checksum, and only
// two radial distortion coefficients per camera, so a run with n_d > 2 (or
// non-zero d3/d4) writes everything else but skips this file with a message.
struct DwecalConfig {
    bool enabled = false;
    // Output resolution. Must be one of UXGA 1600x1200, HD 1280x720,
    // SVGA 800x600, VGA 640x480 -- the format's resolution byte is an enum and
    // nothing else can be represented.
    int output_width = 1600;
    int output_height = 1200;
    // File name inside the timestamped run directory. No path separators: the
    // directory is chosen by the node, the name only by this field.
    std::string basename = "calibration.dwecal";
    // camera1 must be the physically right camera (positive x of its origin in
    // camera0's frame). The file has no field that says which map is which, so
    // a reversed rig would silently produce mirrored disparity. Enforced.
    bool require_camera1_right = true;
};

struct RigConfig {
    BoardConfig board;
    DetectorConfig detector;
    PerformanceConfig performance;
    SolveConfig solve;
    ExternalConfig external;
    FrameConfig frames;
    CheckConfig check;
    OutputConfig output;
    DwecalConfig dwecal;

    // Insertion-ordered by camera name (std::map): "left" before "right".
    std::map<std::string, CameraConfig> cameras;

    ExtrinsicConfig extrinsic;

    // True when the rig declares exactly two cameras, which is what the
    // cross-camera check needs.
    bool two_cameras() const { return cameras.size() == 2; }

    // Throws std::runtime_error with a message naming the offending field if the
    // file cannot be read, does not parse, or is missing something required.
    // Missing optional fields keep the defaults above.
    static RigConfig load(const std::string& path);

    // nullptr when there is no such camera.
    const CameraConfig* camera(const std::string& name) const;

    // The other camera's name, or "" when the rig does not have exactly two.
    std::string other_camera(const std::string& name) const;
};

// Rotation from the config's rx/ry/rz.
//
// The triple is a Rodrigues rotation VECTOR (axis-angle, w), in radians, and
// this returns exp(w^) -- Eigen's AngleAxis is exactly that. It is not an Euler
// triple.
//
// That is what both sources of these numbers use, so numbers copy across
// unmodified:
//
//   - discocal's own se3.rot, which relative_camera_poses.txt prints and
//     LieAlgebra::to_SO3 exponentiates (lib_calib/CLieAlgebra.cpp:45).
//   - the libCalib JSON's transform.rotation, which is what cv::Rodrigues
//     produces and what its readers feed straight back into cv::Rodrigues.
//
// It matters beyond tidiness: the two agree only to first order in |w|. The
// near-degenerate stereo solution in calibration_set_big/extrinsic_result_fullopt
// has rz = 3.127083, where reading the triple as Rz*Ry*Rx would give a
// completely different rotation rather than a slightly different one.
Eigen::Matrix3d rotation_from_rxryrz(double rx, double ry, double rz);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_RIG_CONFIG_H_
