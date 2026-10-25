#ifndef DISCOCAL_ROS_CROSS_CAMERA_CHECK_H_
#define DISCOCAL_ROS_CROSS_CAMERA_CHECK_H_

// Held-out cross-camera check.
//
// The idea: take frames the calibration never saw, predict what the *other*
// camera should have seen in them using only the calibrated intrinsics and the
// fixed extrinsic, and score the prediction against what that camera actually
// detected. Nothing here is fitted, so the check cannot be satisfied by the fit
// -- if the intrinsics are wrong, or the extrinsic is not the real one, the
// residual shows up here and nowhere else.
//
// For a held-out frame the calibrated camera's board pose T (board frame ->
// camera frame) comes from Calibrator::update_Es with its intrinsics held
// constant, which is the same primitive upstream uses for `fix_intrinsic`. Fold
// in the extrinsic and the nominal board projects straight into the other
// camera:
//
//     T_other = T_ext * T                 (both are board -> camera 4x4s)
//     pixel   = BoardProjector::project(lattice[k], T_other, other_params)
//
// which is compared with the other camera's detected dot of the same lattice
// index, using the library's own conic projection so the residual is in the same
// units as the calibration's own (see board_frame.h).
//
// The residual is over all dots of all checked frames, in pixels of the other
// camera's image; `rms_px` is that pooled RMS and `worst_pair_rms_px` is the
// worst single held-out frame. Both are true RMS values, which is what their
// names promise. The threshold they are judged against comes back as `limit_px`
// so a result is readable without the config it was produced under.
//
// Note this is deliberately *not* the library's own residual metric:
// Calibrator::cal_reprojection_error reports a mean of mean distances, and only
// the reported `reprojection_error` reproduces that (see board_frame.h) so it
// stays comparable with the offline baseline.

#include <cstdint>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params, ::Shape

namespace discocal_ros {

struct HeldOutFrame {
    std::int64_t stamp_ns = 0;
    // This camera's board pose, board frame -> camera frame.
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // This camera's own detections for the frame, used only to sanity-check the
    // pose above before that pose is trusted to judge anything.
    std::vector<::Shape> own_shapes;
    // The other camera's detections for the same instant.
    std::vector<::Shape> other_shapes;

    // External-pose mode only. The measured dot positions, in label order, are
    // what evaluate() needs to solve this frame's pose against the final
    // intrinsics; in image mode they stay empty/false. `T` above is still the
    // board pose the check consumes -- external evaluation writes it from the
    // matched rigid fit.
    bool has_external = false;
    bool other_has_external = false;
    std::vector<Eigen::Vector3d> own_external_points;
    std::vector<Eigen::Vector3d> other_external_points;
};

struct CheckResult {
    // False when the check could not run at all: no extrinsic, the other
    // camera's intrinsics unknown, or no usable pairs. Not a verdict on the
    // calibration -- the caller decides whether that is fatal.
    bool ran = false;
    bool passed = false;
    // Why it did not run, or which limit it failed. Human-readable.
    std::string reason;
    int pairs = 0;            // held-out frames actually scored
    int points = 0;           // dot correspondences behind the RMS
    double rms_px = 0.0;      // over all dots of all pairs
    // Worst single pair. Named apart from the limit on purpose: this struct and
    // the function below once both called it max_rms_px, one meaning the worst
    // measurement and the other the threshold, and the action result ended up
    // publishing the measurement under a name the contract called a limit.
    double worst_pair_rms_px = 0.0;
    double limit_px = 0.0;    // what `rms_px` was compared against
    // Held-out frames whose own pose did not reproduce their own camera's
    // detections, so they were not used to judge anything. Non-zero means the
    // held-out pose solve diverged on those frames -- a solver problem,
    // deliberately not reported as an extrinsic problem.
    int pairs_dropped = 0;
    int pairs_unpaired = 0;  // held-out frames the other camera did not detect
};

// A pose whose own-frame reprojection exceeds this is treated as a failed solve
// and its pair is dropped rather than counted. Generous on purpose: the fitted
// set reaches ~0.24 px and acceptance is ~1 px, so 5 px is far past anything a
// real intrinsic/extrinsic mismatch produces and into "this pose is simply
// wrong" territory.
extern const double kPoseSanityRmsPx;

// `T_own_to_other` is the rigid transform taking a point from the calibrated
// camera's frame into the other camera's: `p_other = T_own_to_other * p_own`.
// The session builds it from the extrinsic, in whichever direction this camera
// sits. `lattice` is board_lattice(board); `radius` is board.radius (metres);
// `n_d` sizes the projector's tables; `mode` is the calibrator's own.
// `limit_px` is check.max_rms_px from the rig config.
CheckResult run_cross_camera_check(const std::vector<HeldOutFrame>& frames,
                                   const std::vector<Eigen::Vector3d>& lattice,
                                   double radius, int n_d, int mode,
                                   const ::Params& own_params,
                                   const ::Params& other_params,
                                   const Eigen::Matrix4d& T_own_to_other,
                                   double limit_px);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_CROSS_CAMERA_CHECK_H_
