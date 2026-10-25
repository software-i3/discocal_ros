#ifndef DISCOCAL_ROS_BOARD_FRAME_H_
#define DISCOCAL_ROS_BOARD_FRAME_H_

// One camera's detection of the board in one image, plus the nominal board the
// detection is indexed against, plus the projection that connects them.

#include <cstdint>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Shape

// Vendored; forward-declared so its header -- which does `using namespace std;`
// at global scope -- stays out of this one.
class MomentsTracker;

namespace discocal_ros {

// The externally-supplied dot positions for one frame, in the camera's optical
// frame and indexed by the dot's original label. This is what turns the board
// pose from an unknown into a measurement: the solver reprojects the nominal
// lattice through the pose fitted from these points instead of estimating a
// pose from the pixels.
//
// `present` is false when no pose sample was available (no topic, a stamp
// outside sync_tolerance_s, an unusable message). `error` then says why, and
// the session counts the frame as pose-missing rather than silently solving
// with fewer frames.
struct ExternalPoints {
    bool present = false;
    // True when the sample's points were linearly interpolated between two
    // bracketing messages at this frame's stamp. False for an exact/nearest
    // sample.
    bool interpolated = false;
    // Stamp of the message the points came from (the earlier one when
    // interpolated).
    std::int64_t source_stamp_ns = 0;
    // |frame stamp - source stamp| in seconds; for an interpolated sample, the
    // distance to the nearer bracket.
    double dt_s = 0.0;
    std::string frame_id;
    std::string error;
    // One position per label. Size == n_x*n_y for a usable message.
    std::vector<Eigen::Vector3d> points;
};

struct BoardFrame {
    std::string camera;
    // header.stamp in nanoseconds. The two cameras of this rig are stamped from
    // one source message, so pairing is exact equality on this value -- there is
    // no tolerance to tune.
    std::int64_t stamp_ns = 0;
    // The detector's dots, in the order TargetDetector::detect returns them.
    std::vector<::Shape> shapes;
    // Populated whenever the camera has a point_pose_topic and a sample was
    // found for this frame; only solve.pose_source == "external" consumes it.
    // Attaching it regardless lets a per-goal override switch sources without
    // restarting the node, and lets the cross-camera check use it when the
    // other camera is not the one being solved.
    ExternalPoints external;
};

// The board's dots in board-frame metres, in the same order the detector
// returns them: index k here and index k of a detection are the same physical
// dot. That index pairing is the library's own assumption -- it is what
// CalibrationFunctor relies on when it zips origin_target with a detection --
// and it is the one thing the cross-camera check inherits from the library.
//
// Mirrors Calibrator::set_origin_target (lib_calib/CCalibrator.cpp:77). If that
// ever changes, this must change with it.
std::vector<Eigen::Vector3d> board_lattice(const BoardConfig& board);

// Projects the nominal board the way the library does, so that anything scored
// through it is scored in the library's own units.
//
// This matters more than it looks. MomentsTracker::project in mode 0 does not
// project the dot's centre as a point: it maps the circle's world conic through
// the pose, then takes the *centroid of the resulting distorted ellipse*
// (ne2dp). Those two differ by more than the residuals being measured here --
// an ellipse's image centroid is not the image of its centre -- and the detector
// reports moment centroids, so the library's whole model is built around the
// conic path. Projecting a point instead would put a systematic offset of the
// same order as the effect being tested into every residual. Mode 2 (square
// boards, corners) is a genuine point projection, and the same entry point
// handles both.
//
// One tracker is built per projector because its construction is not cheap;
// reuse it across frames rather than constructing one per call.
class BoardProjector {
public:
    explicit BoardProjector(int n_d);
    ~BoardProjector();

    // `T` is board frame -> whatever camera `params` describes, as a 4x4 with
    // the usual [R|t] layout. `mode` is the calibrator's own: 0 for a circle
    // board, 2 for a square one.
    Eigen::Vector2d project(double wx, double wy, double radius, const Eigen::Matrix4d& T,
                            const ::Params& params, int mode) const;

    // Every lattice dot through one pose. The symmetry matcher scores one
    // candidate per permutation per frame, so this is the hot loop; one
    // projector (and one MomentsTracker) is reused across all of them.
    std::vector<Eigen::Vector2d> project_all(const std::vector<Eigen::Vector3d>& lattice,
                                             double radius, const Eigen::Matrix4d& T,
                                             const ::Params& params, int mode) const;

    // The library's own calibration residual, reproduced from its private
    // Calibrator::cal_reprojection_error (lib_calib/CCalibrator.cpp:447):
    //
    //     mean over frames of ( mean over dots of |projected - detected| )
    //
    // It is a mean of mean distances, NOT a root-mean-square, and it is the
    // number the library prints as "Reprojection error". Reproduced here rather
    // than reported differently so this node's number is comparable with the
    // offline baseline's 0.241 px.
    //
    // Frames whose detection count does not match the lattice are skipped.
    double reprojection_error(const std::vector<Eigen::Matrix4d>& T_board_to_camera,
                              const std::vector<std::vector<::Shape>>& detections,
                              const std::vector<Eigen::Vector3d>& lattice, double radius,
                              const ::Params& params, int mode) const;

private:
    BoardProjector(const BoardProjector&);
    BoardProjector& operator=(const BoardProjector&);

    MomentsTracker* tracker_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_BOARD_FRAME_H_
