#ifndef DISCOCAL_ROS_CALIBRATE_SESSION_H_
#define DISCOCAL_ROS_CALIBRATE_SESSION_H_

// One calibration run for one camera: collect gated frames, solve, check the
// result against the other camera through the fixed extrinsic, write the
// camera_info.
//
// ROS-free and image-free on purpose. The node decodes images and runs the
// detector on its own thread and hands detections in here; this class never sees
// a cv::Mat. That keeps the whole policy -- gating, holdout split, retry,
// thresholds -- testable without a roscore, and it keeps detection on one thread
// (see the note about TargetDetector's shared debug counter in node.cpp).

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "discocal_ros/board_frame.h"
#include "discocal_ros/cross_camera_check.h"
#include "discocal_ros/diversity_gate.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params

namespace discocal_ros {

struct SessionStats {
    // Detections offered for the camera being calibrated.
    int detected = 0;
    // Of those, how many passed the pose-diversity gate.
    int accepted = 0;
    // Rejected by the gate because the pose was too close to an accepted one,
    // or because its pose signature was not computable.
    int duplicate = 0;
    // Accepted frames the other camera has no detection for at the same stamp.
    // Only meaningful once solve() (or finalize()) has run; until then it counts
    // what was known at the moment each frame was accepted, so it can only fall.
    int unpaired = 0;
    // External-pose mode: detections that arrived with no usable pose sample
    // for their stamp (nothing on the topic, or outside sync_tolerance_s), and
    // detections whose sample existed but could not be used (wrong dot count,
    // wrong frame_id, a non-finite or non-positive-z position). Both are
    // counted before the diversity gate, so they never enter `accepted`.
    int pose_missing = 0;
    int pose_invalid = 0;
};

struct SessionResult {
    bool success = false;
    std::string message;

    // The solved intrinsics. Only meaningful when `success` is true: on a failed
    // check they are left populated so the rejected attempt can be logged, and
    // must not be published or written anywhere.
    ::Params params;
    int n_frames = 0;          // accepted frames that went into the solve
    int n_frames_heldout = 0;  // accepted frames reserved for the check
    int attempts = 0;          // 1 on the first try, 2 on the retry
    double reprojection_error = 0.0;
    CheckResult check;
    std::string camera_info_path;      // "" when nothing was written
    std::string raw_output_directory;  // where the library wrote its own results
    double fit_seconds = 0.0;

    // Which path produced this result, and the external-path diagnostics. In
    // image mode these stay at their defaults ("image", zeros).
    std::string pose_source = "image";
    int frames_matched = 0;       // external mode: frames that entered the fit
    int frames_ambiguous = 0;     // external mode: symmetry margin below the limit
    int frames_rejected_external = 0;  // external mode: not matched for any reason
    double mean_rigidity_rms_m = 0.0;
};

// The result of the mono solve alone, before any cross-camera verdict. Kept
// separate so the both-camera orchestration can evaluate a camera against the
// *other* camera's final intrinsics (and after a joint refinement) instead of
// whatever was available when the mono solve started.
// Evenly spaced positions in [0, count), so a held-out set spans the
// collection instead of clustering at one end. Deterministic, sorted and
// deduplicated. Shared with the both-camera orchestrator, which must pick the
// same positions from both cameras' paired stamps.
std::vector<int> spread_indices(int count, int n_holdout);

struct SolveOutcome {
    bool ok = false;  // params are usable (finite, positive fx/fy)
    std::string message;
    ::Params params;

    int n_frames = 0;
    int n_frames_heldout = 0;
    double reprojection_error = 0.0;
    double fit_seconds = 0.0;
    std::string raw_output_directory;

    // The fit set, in solve order. `fit_poses` are board-to-camera transforms
    // from the calibrator, one per entry.
    std::vector<std::int64_t> fit_stamps;
    std::vector<std::vector<::Shape> > fit_shapes;
    std::vector<Eigen::Matrix4d> fit_poses;

    // External-path diagnostics, mirroring SessionResult.
    std::string pose_source = "image";
    int matched_frames = 0;
    int ambiguous_frames = 0;
    int rejected_external_frames = 0;
    double mean_rigidity_rms_m = 0.0;
    std::string initialization;

    // The held-out frames with their detections. `T` is filled by evaluate(),
    // which re-solves the poses against the final intrinsics -- after a joint
    // refinement the mono poses are no longer the right ones.
    std::vector<HeldOutFrame> holdout_frames;

    // Why no frames were held out, when that is the case.
    std::string holdout_reason;
};

class CalibrateSession {
public:
    // `known_intrinsics` is what the node already has for each camera: the
    // config's `file` cameras plus anything calibrated earlier in this process.
    // It is what makes the check possible for a camera whose partner is not
    // being calibrated right now, and it is deliberately a snapshot -- one
    // session is one goal.
    CalibrateSession(const RigConfig& rig, const std::string& camera, int budget,
                     const std::map<std::string, ::Params>& known_intrinsics);

    // Needed before the first on_detection: the gate normalises by image size.
    void set_image_size(int width, int height);
    bool has_image_size() const { return width_ > 0 && height_ > 0; }
    int image_width() const { return width_; }
    int image_height() const { return height_; }

    // A detection of the board by the camera being calibrated. Returns true once
    // the accepted-frame budget has been reached.
    bool on_detection(const BoardFrame& frame);

    // A detection by the other camera. Only frames sharing a stamp with an
    // accepted frame are ever looked at, so this is cheap to call for every
    // frame the other camera produces.
    void on_other_detection(const BoardFrame& frame);

    bool budget_reached() const { return stats_.accepted >= budget_; }
    int accepted() const { return stats_.accepted; }
    int budget() const { return budget_; }
    const SessionStats& stats() const { return stats_; }

    // Accepted frames in arrival order. Read-only; the both-camera orchestrator
    // uses the stamps to choose a holdout shared by both cameras.
    const std::vector<BoardFrame>& accepted_frames() const { return accepted_; }

    // The calibrator's own floor: Calibrator::cal_initial_params needs at least
    // 5 scenes, and the library's drivers require at least 4 + n_d.
    int min_fit_frames() const;

    // What happened to the most recent on_detection, for the feedback/log line.
    const std::string& last_decision() const { return last_decision_; }

    // Solve, check, and write. May be called again after reset() -- it holds no
    // state that a second call would corrupt, beyond spending an attempt.
    //
    // This is the mono goal's entry point. It is written in terms of solve(),
    // evaluate() and write_camera_info() so the both-camera orchestration can
    // reuse the same stages without duplicating policy.
    SessionResult finalize(bool allow_unchecked);

    // --- stages, for the both-camera orchestrator --------------------------
    //
    // `forced_holdout`, when non-null, is a stamp list shared with the other
    // camera; only stamps that were accepted here are held out, and
    // `frames.holdout_fraction` is ignored. When null the holdout is selected
    // exactly as finalize() always did: only when the check is configured and
    // the other camera's intrinsics are already known.
    //
    // `attempt` only names the raw output subdirectory; the caller owns the
    // attempt counter.
    SolveOutcome solve(const std::vector<std::int64_t>* forced_holdout, int attempt);

    // Re-solves the held-out poses with `final_params` and scores them against
    // the other camera's intrinsics and the fixed extrinsic. Returns a
    // CheckResult with ran=false and a reason when the check cannot run.
    CheckResult evaluate(const SolveOutcome& outcome, const ::Params& final_params) const;

    // The camera_info document for these params, with the config's frame_id
    // (falling back to the camera name). Exposed so the both-camera
    // orchestrator can stage both files and rename them together.
    std::string camera_info_yaml(const ::Params& params) const;

    // Writes <directory>/<camera>_camera_info.yaml. `directory` is created if
    // needed. On success `path` names the file. The file is written atomically.
    bool write_camera_info(const ::Params& params, const std::string& directory,
                           std::string* path, std::string* error) const;

    // The other camera's intrinsics, when they were not known at construction
    // (the both-camera case: they only exist after both mono solves, and again
    // after joint refinement).
    void set_other_params(const ::Params& params);

    // Where discocal_raw/<camera>/attempt_N goes. Defaults to the rig config's
    // output directory; the both-camera orchestrator points it at the run
    // directory it just created.
    void set_output_directory(const std::string& directory);

    // Drop the collected frames and forget the gate's signatures so a retry
    // starts a fresh collection. Attempts already spent are kept.
    void reset();

    int attempts() const { return attempts_; }
    const std::string& camera() const { return camera_; }
    const std::string& other_camera() const { return other_camera_; }

    // Whether a cross-camera check can run right now: `check` enabled, the
    // camera is one half of the configured extrinsic, and the other camera's
    // intrinsics are known. When false, `check_unavailable_reason()` says which
    // of those is missing.
    bool check_configured() const { return check_configured_ && have_other_params_; }
    const std::string& check_unavailable_reason() const { return check_unavailable_reason_; }

private:
    const RigConfig rig_;
    std::string camera_;
    std::string other_camera_;
    int budget_;
    int width_ = 0;
    int height_ = 0;
    int attempts_ = 0;

    // True when check.enabled, the extrinsic exists, the rig declares exactly
    // two cameras and this camera is one end of the extrinsic. Whether the
    // other camera's intrinsics are known is tracked separately: in both-camera
    // mode they only exist after both mono solves.
    bool check_configured_ = false;
    bool have_other_params_ = false;
    std::string check_unavailable_reason_;
    ::Params other_params_;
    // Board-to-camera transform of the extrinsic, oriented so that it maps this
    // camera's frame into the other camera's.
    bool have_other_transform_ = false;
    Eigen::Matrix4d T_own_to_other_ = Eigen::Matrix4d::Identity();

    std::string output_directory_;
    void update_check_unavailable_reason();

    DiversityGate gate_;
    SessionStats stats_;
    std::string last_decision_;

    // Accepted frames for the camera being calibrated, in arrival order.
    std::vector<BoardFrame> accepted_;
    // The other camera's frames, by stamp, pruned to a bounded history: an
    // accepted frame and the other camera's frame for the same instant arrive
    // within milliseconds of each other, so a few seconds of slack is ample.
    // The whole frame, not only the shapes, because external-mode evaluation
    // also needs that camera's measured dot positions.
    std::map<std::int64_t, BoardFrame> other_;
    std::deque<std::int64_t> other_order_;
    static const std::size_t kOtherHistory = 600;

    // External-pose path. `fit_positions` and `holdout_positions` index
    // accepted_ and are selected exactly as in the image path.
    SolveOutcome solve_external(const std::vector<int>& fit_positions,
                                const std::vector<int>& holdout_positions, int attempt);
    CheckResult evaluate_external(const SolveOutcome& outcome,
                                  const ::Params& final_params) const;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_CALIBRATE_SESSION_H_
