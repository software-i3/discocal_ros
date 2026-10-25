#ifndef DISCOCAL_ROS_RIG_SESSION_H_
#define DISCOCAL_ROS_RIG_SESSION_H_

// Both-camera calibration: collect frames for every camera, solve the two
// intrinsic sets independently, optionally refine them jointly with the
// configured extrinsic held bit-constant, then check each camera against the
// other on a holdout shared by construction, and write one timestamped run
// directory.
//
// The shared holdout is the correctness-critical part. If either camera fitted
// a frame that the cross-camera check then scores, the check is no longer an
// independent measurement. choose_common_holdout() intersects the two accepted
// stamp sets first and only ever holds out stamps present in both.
//
// ROS-free, like CalibrateSession. The node owns the image pipeline; this class
// only sees detections.

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "discocal_ros/board_frame.h"
#include "discocal_ros/calibrate_session.h"
#include "discocal_ros/performance_report.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params

namespace discocal_ros {

struct CameraOutcome {
    std::string camera;

    // `result.params` is populated whenever the mono solve produced usable
    // intrinsics -- including when the cross-camera check failed, because the
    // run directory still records the rejected calibration. `success` is the
    // full per-camera verdict (solve + check + artifact writes).
    SessionResult result;

    // True when a joint refinement replaced this camera's intrinsics but the
    // reported sigmas are still the mono solve's: StereoCalibrator estimates no
    // covariance.
    bool uncertainties_from_mono_stage = false;
};

struct RigSessionResult {
    bool success = false;
    std::string message;
    int attempts = 0;

    // "image" or "external", from rig.solve.pose_source. In external mode the
    // joint refinement is not applicable and is skipped; the per-camera
    // diagnostics in CameraOutcome::result describe the matching instead.
    std::string pose_source = "image";

    bool joint_refined = false;
    double joint_fit_seconds = 0.0;

    // Wall and worker-thread CPU for each solve stage, measured where the work
    // happens. Empty stage entries mean the stage did not run (e.g. no joint
    // refinement, or a camera whose mono solve failed before evaluation).
    RigTimings timings;

    // One entry per camera, in the order supplied to the constructor (the node
    // passes std::map order, so "left" precedes "right").
    std::vector<CameraOutcome> cameras;

    // The timestamped run directory and its artifacts. All are "" until solve()
    // has created the directory / written the file.
    std::string run_directory;
    std::string land_directory;
    std::string dwecal_path;
    std::string dwecal_sha256;

    // Non-fatal problems: a joint refinement that fell back to mono, a skipped
    // .dwecal, a holdout too small to check. Fatal problems go in `message`
    // with `success == false`.
    std::vector<std::string> warnings;
};

// The stamps held out for the cross-camera check, given the two cameras'
// accepted stamps. Only stamps accepted by both cameras can be held out.
//
// `fraction` is the requested share of the paired stamps; the result is also
// clamped so each camera keeps at least `min_fit_a` / `min_fit_b` frames for
// its own solve. Positions are chosen by spread_indices() over the sorted,
// deduplicated paired list, so the same inputs always return the same stamps.
// An empty result means no stamp can be held out (no pairs, fraction 0, or the
// fit floor leaves no room).
std::vector<std::int64_t> choose_common_holdout(
    const std::vector<std::int64_t>& accepted_a,
    const std::vector<std::int64_t>& accepted_b, double fraction, int min_fit_a,
    int min_fit_b);

// Creates <base>/calibration_YYYYmmdd_HHMMSS, or _1/_2/... when that name
// already exists. `base` is created if needed. The name is not reused, so two
// runs in the same second cannot overwrite each other.
bool create_timestamped_run_directory(const std::string& base, std::string* directory,
                                      std::string* error);

class RigSession {
public:
    // `cameras` must name exactly two configured `source: calibrate` cameras.
    // `known_intrinsics` has the same meaning as in CalibrateSession; both-mode
    // goals pass an empty map because neither camera's result exists yet.
    // Throws std::runtime_error naming the problem for an invalid camera list.
    RigSession(const RigConfig& rig, const std::vector<std::string>& cameras, int budget,
               const std::map<std::string, ::Params>& known_intrinsics);

    // One image size per camera, learned from its first decoded frame. The
    // diversity gate and the matrix scaling both need it.
    void set_image_size(const std::string& camera, int width, int height);
    bool has_image_size(const std::string& camera) const;

    // Routes a detection to its owning camera's session and, as the other
    // camera's detection, to every other session. Returns true once every
    // camera has reached its accepted-frame budget.
    bool on_detection(const BoardFrame& frame);

    bool budget_reached() const;
    const SessionStats& stats(const std::string& camera) const;

    // The calibrator's own floor for this camera (see
    // CalibrateSession::min_fit_frames). Below it the solve cannot run at all,
    // so it is the only accepted-frame count that makes a short collection a
    // failure rather than a smaller calibration.
    int min_fit_frames(const std::string& camera) const;

    // Solve both cameras, optionally refine jointly, evaluate, and write the
    // run directory. May be called again after reset(): the run directory (and
    // the attempt counter) stay, so retries line up under one calibration
    // timestamp.
    //
    // `interrupt`, when set, is polled between stages (after the mono solves,
    // before the joint refine, before evaluation and before writing). A true
    // return stops the solve with success=false and a message; whatever
    // artifacts were not yet written are simply absent. Stages already running
    // -- in particular a Ceres solve -- are never interrupted midway.
    typedef std::function<bool()> InterruptCheck;
    RigSessionResult solve(bool allow_unchecked,
                           const InterruptCheck& interrupt = InterruptCheck());

    // Drop collected frames for a retry. The run directory and attempt count
    // are kept deliberately.
    void reset();

    int attempts() const { return attempts_; }
    const std::vector<std::string>& cameras() const { return cameras_; }

private:
    struct CameraState {
        std::unique_ptr<CalibrateSession> session;
        SolveOutcome solved;
        ::Params final_params;
        bool has_final_params = false;
        bool uncertainties_from_mono_stage = false;
        CheckResult check;
    };

    CameraState& state(const std::string& camera);
    const CameraState& state(const std::string& camera) const;

    // {parent, child} when the configured extrinsic names both cameras, else
    // empty. This is the only order in which a joint refinement and a land /
    // .dwecal conversion are meaningful.
    std::vector<std::string> parent_child_order() const;

    const RigConfig rig_;
    std::vector<std::string> cameras_;
    std::map<std::string, CameraState> states_;
    int budget_;
    int attempts_ = 0;

    std::string run_directory_;
    std::string started_at_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_RIG_SESSION_H_
