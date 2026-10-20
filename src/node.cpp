// discocal_ros node: subscribe to one or two image feeds, collect board
// detections under a pose-diversity gate, calibrate one camera (or both with a
// fixed extrinsic) on request, check the result, and return it through an
// action.
//
// Threading. Image decoding runs on the dedicated image CallbackQueue's
// AsyncSpinner (`image_spinner_`), so a slow decode never blocks the action
// server on the global queue. Those callbacks do nothing but enqueue, and only
// while `collecting_` is true -- a goal that is solving drops messages before
// decoding them. One worker thread owns the detectors, the calibration session
// and the known-intrinsics map, and does all the real work. Detection must not
// be spread over several threads even though it looks parallelisable:
// TargetDetector::detect_circles keeps its per-frame dump counter in a plain
// `static int dbg_call` (CTargetDetector.cpp), incremented without a lock, so
// concurrent detections race on it and the per-frame debug dumps come out
// interleaved. The worker is also what makes the session safe to use without a
// mutex -- nothing else touches it.
//
// While a goal is being solved the worker is blocked and frames are not
// decoded at all (collecting_ is false). The queue is drained before each
// collection, so a stale backlog can never leak into the next attempt.

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/resource.h>
#include <sys/stat.h>

#include <omp.h>
#include <opencv2/core.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <actionlib/server/action_server.h>
#include <discocal_ros/CalibrateCameraAction.h>

#include "discocal_ros/calibrate_session.h"
#include "discocal_ros/image_source.h"
#include "discocal_ros/performance_report.h"
#include "discocal_ros/rig_config.h"
#include "discocal_ros/rig_session.h"

// The detector's header does `using namespace std;` and pulls in OpenCV
// unconditionally, so it stays in this translation unit. That is also why the
// detectors are held as unique_ptrs in a map rather than by value.
#include "CTargetDetector.h"

namespace discocal_ros {
namespace {

// `Action` is the generated action spec; `ActionServer` is the server over it.
// The message types are reached through the *server* (ActionServer::Goal,
// ::Result, ::Feedback): those come from actionlib's ACTION_DEFINITION macro on
// the ActionServer base class. The generated action class itself only has
// _action_goal_type / _action_result_type / _action_feedback_type, so
// `Action::Result` does not name anything.
//
// ActionServer, not SimpleActionServer, on purpose: a goal arriving while
// another is solving must not become the server's single "current goal" before
// the running one is finished. With per-goal handles the running goal can be
// terminated (PREEMPTED) on its own handle, and the replacement then starts
// from the queue. With SimpleActionServer, acceptNewGoal() cancels the running
// goal immediately and makes the new goal current; the old goal's later
// setPreempted() then cancels the replacement, which is exactly the bug the
// preemption test caught.
typedef discocal_ros::CalibrateCameraAction Action;
typedef actionlib::ActionServer<Action> ActionServer;

// Bound on queued decoded frames. At 30 Hz that is about a second of slack; a
// collection needs the board visible in a run of frames anyway, and anything
// older has been superseded.
const std::size_t kMaxQueuedFrames = 30;

// How long the worker waits for work before re-checking its deadlines. It only
// bounds how promptly a deadline, a preempt or a shutdown is noticed; it is not
// a frame rate limit.
const int kWorkerTickMs = 100;

const char* const kStateCollecting = "COLLECTING";
const char* const kStateCalibrating = "CALIBRATING";
const char* const kStateRetrying = "RETRYING";
const char* const kStateDone = "DONE";
const char* const kStateFailed = "FAILED";

bool both_mode(const std::string& camera) {
    return camera == "both" || camera == "all";
}

std::string number_text(double value) {
    std::ostringstream oss;
    oss.precision(17);
    oss << value;
    return oss.str();
}

void set_environment(const char* name, const std::string& value) {
    if (setenv(name, value.c_str(), 1) != 0) {
        ROS_WARN("discocal_ros: could not set %s=%s; the library will use its own default", name,
                 value.c_str());
    }
}

// The vendored sources read their tuning from the environment. They cache some
// of it in function-local statics on first use, so this must run in the node
// constructor -- before the first detector is built and before the first solve.
//
// Numeric detector values of 0 mean "leave the library default", which is an
// *unset* variable, not a literal zero. DISC_DEBUG_PROBE is presence-based, so
// false must unset it too; DISC_EDGE_STATS is value-based and explicit 0 is
// fine.
void apply_environment(const RigConfig& rig) {
    const DetectorConfig& det = rig.detector;
    const PerformanceConfig& perf = rig.performance;

    if (det.size_threshold_px2 > 0.0) {
        set_environment("DISC_SIZE_THRESHOLD", number_text(det.size_threshold_px2));
    } else {
        unsetenv("DISC_SIZE_THRESHOLD");
    }
    if (det.dist_threshold_px > 0.0) {
        set_environment("DISC_DIST_THRESHOLD", number_text(det.dist_threshold_px));
    } else {
        unsetenv("DISC_DIST_THRESHOLD");
    }
    if (det.max_candidates > 0) {
        set_environment("DISC_MAX_CANDIDATES", std::to_string(det.max_candidates));
    } else {
        unsetenv("DISC_MAX_CANDIDATES");
    }
    if (det.edge_contrast > 0.0) {
        set_environment("DISC_EDGE_CONTRAST", number_text(det.edge_contrast));
    } else {
        unsetenv("DISC_EDGE_CONTRAST");
    }
    if (det.edge_inner_uniformity > 0.0) {
        set_environment("DISC_EDGE_INNER_UNIF", number_text(det.edge_inner_uniformity));
    } else {
        unsetenv("DISC_EDGE_INNER_UNIF");
    }
    if (det.edge_outer_uniformity > 0.0) {
        set_environment("DISC_EDGE_OUTER_UNIF", number_text(det.edge_outer_uniformity));
    } else {
        unsetenv("DISC_EDGE_OUTER_UNIF");
    }
    if (det.probe) {
        set_environment("DISC_DEBUG_PROBE", "1");
    } else {
        unsetenv("DISC_DEBUG_PROBE");
    }
    set_environment("DISC_EDGE_STATS", det.edge_stats ? "1" : "0");

    set_environment("DISC_SPARSE_INFO", perf.sparse_info ? "1" : "0");
    set_environment("DISC_BLOCK_COV", perf.block_cov ? "1" : "0");
    set_environment("DISC_SPARSE_SCHUR", perf.sparse_schur ? "1" : "0");
    set_environment("DISC_UNC_DUMP", perf.unc_dump ? "1" : "0");
    set_environment("DISC_TIME", perf.time ? "1" : "0");
    if (perf.ceres_threads > 0) {
        set_environment("DISC_CERES_THREADS", std::to_string(perf.ceres_threads));
    } else {
        unsetenv("DISC_CERES_THREADS");
    }
    if (perf.detector_threads > 0) {
        set_environment("OMP_NUM_THREADS", std::to_string(perf.detector_threads));
    } else {
        unsetenv("OMP_NUM_THREADS");
    }
}

}  // namespace

class DiscocalNode {
public:
    DiscocalNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DiscocalNode();

private:
    struct PendingFrame {
        std::string camera;
        std::int64_t stamp_ns = 0;
        cv::Mat bgr;
    };

    // --- callbacks (ros::spin thread) --------------------------------------
    void onGoal(ActionServer::GoalHandle handle);
    void onCancel(ActionServer::GoalHandle handle);
    static void onFrame(void* user, const std::string& camera, std::int64_t stamp_ns,
                        const cv::Mat& bgr);

    // --- worker ------------------------------------------------------------
    void workerLoop();
    // Runs one goal to completion: collect, solve, check, retry as configured.
    // Blocks the worker for the duration, on purpose -- see the file comment.
    void runGoal(const boost::shared_ptr<const ActionServer::Goal>& goal);
    // The both-camera variant. `cameras` is in rig-config (std::map) order.
    void runRigGoal(const boost::shared_ptr<const ActionServer::Goal>& goal,
                    const std::vector<std::string>& cameras);
    // The frame is already BGR. True when the board was found in it.
    bool processFrame(const PendingFrame& frame);
    bool processRigFrame(const PendingFrame& frame);
    // Discards everything already queued, so a collection starts on live frames
    // rather than on a backlog from before the last solve.
    void drainQueue();
    // Waits up to `timeout_ms` for the next frame. False on timeout.
    bool popFrame(PendingFrame* frame, int timeout_ms);
    // True once a new goal has arrived, which ends the one in progress.
    bool goalPending();
    // Terminal transitions for the goal the worker is running. Each takes the
    // handle copy under the lock and calls actionlib outside it. The running
    // goal is the only one these can touch; a replacement is a different
    // handle and is not affected.
    void succeedGoal(const ActionServer::Result& result, const std::string& text);
    void abortGoal(const ActionServer::Result& result, const std::string& text);
    void preemptGoal(const std::string& text);

    void publishFeedback(const std::string& state, const CalibrateSession* session, int budget,
                         bool force_log);
    void publishRigFeedback(const std::string& state, int budget, bool force_log);
    // Fills both the legacy scalar result fields and the one-element cameras[]
    // that mirrors them. The camera name is not in SessionResult, so it is
    // passed in.
    void fillResult(const std::string& camera, const SessionResult& result,
                    ActionServer::Result* out) const;
    void fillCameraCalibration(const std::string& camera, const SessionResult& result,
                               bool uncertainties_from_mono_stage,
                               CameraCalibration* out) const;
    // Both-mode result: cameras[] per camera, legacy intrinsic scalars zeroed,
    // legacy check scalars describing the worst camera.
    void fillRigResult(const RigSessionResult& result, ActionServer::Result* out) const;
    // True once the goal has been preempted or replaced. Safe to call from the
    // worker inside a solve; takes the same lock as the callbacks.
    bool stopRequested();
    std::string intrinsicsSummary(const ::Params& params) const;
    // Everything the session may use as the *other* camera's intrinsics: every
    // `file` camera from the config, plus anything this process has calibrated.
    // Rebuilt per goal so a calibration is never checked against itself.
    // Worker-thread only.
    std::map<std::string, ::Params> knownIntrinsics() const;

    ros::NodeHandle nh_;
    RigConfig rig_;

    ActionServer server_;
    // Image callbacks run on their own queue so decoding never delays the
    // action server; ros::spin() in main serves the global queue.
    ros::CallbackQueue image_queue_;
    std::vector<std::unique_ptr<ImageSource> > sources_;
    std::unique_ptr<ros::AsyncSpinner> image_spinner_;
    std::map<std::string, std::unique_ptr<TargetDetector> > detectors_;

    // Written by the callbacks, read by the worker.
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<PendingFrame> queue_;
    // The newest accepted goal that has not started yet. A later goal replaces
    // it (the older client is told it was superseded).
    boost::shared_ptr<ActionServer::GoalHandle> pending_goal_;
    // The goal the worker is running, if any. Only the worker writes it; the
    // callbacks read it under mu_ to tell a cancel of the running goal from a
    // cancel of a queued one.
    boost::shared_ptr<ActionServer::GoalHandle> active_goal_;
    bool preempt_requested_ = false;
    bool stop_ = false;

    std::thread worker_;

    // Written by the image callbacks, read by the worker. False outside a
    // collection, which is what lets the callbacks drop messages before decode.
    std::atomic<bool> collecting_{false};

    // Worker-thread only, so no lock guards them.
    std::unique_ptr<CalibrateSession> session_;
    std::unique_ptr<RigSession> rig_session_;
    std::map<std::string, ::Params> calibrated_;  // by camera, from successful goals
    int frames_seen_ = 0;
    int frames_detected_ = 0;
    std::map<std::string, int> rig_frames_seen_;
    std::map<std::string, int> rig_frames_detected_;
    // Goal-scoped; written into the run directory as performance.yaml.
    PerformanceReport performance_;
    // Last accepted count logged, per camera; -1 forces the next line.
    std::map<std::string, int> last_logged_accepted_;
    // The other camera's frames, which the feedback's counters deliberately
    // exclude. Without these the only visible symptom of a starved or unhealthy
    // second feed is the cross-camera check quietly running on nothing.
    int other_seen_ = 0;
    int other_detected_ = 0;
};

DiscocalNode::DiscocalNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), server_(pnh, "calibrate_camera", false) {
    std::string rig_path;
    pnh.getParam("rig_config", rig_path);
    if (rig_path.empty()) {
        throw std::runtime_error(
            "discocal_ros: the ~rig_config parameter is required and must point at a rig YAML "
            "(see config/rig_example.yaml)");
    }
    rig_ = RigConfig::load(rig_path);

    // Before anything can read them: the vendored detector's tuning knobs and
    // the calibrator's performance switches are environment variables, and some
    // are cached on first use. opencv_threads and detector_threads also have a
    // runtime API, applied here so the first decode/detection already obeys them.
    apply_environment(rig_);
    // cv::setNumThreads is process-global, so the constructor is the right
    // place for it. The detector's OpenMP limit is not: nthreads-var is a
    // per-thread ICV, and detection runs on the worker thread, so
    // TargetDetector::set_num_threads() is called there instead.
    cv::setNumThreads(rig_.performance.opencv_threads);
    ROS_INFO("discocal_ros: OpenCV threads: %d (config %d)", cv::getNumThreads(),
             rig_.performance.opencv_threads);

    // The board geometry and the frame policy are the config's; only the output
    // location is worth overriding from the command line, because it is the one
    // thing that differs per machine.
    std::string output_dir;
    pnh.param<std::string>("output_dir", output_dir, "");
    if (!output_dir.empty()) {
        rig_.output.directory = output_dir;
        ROS_INFO("discocal_ros: output directory overridden to %s", output_dir.c_str());
    }

    ROS_INFO("discocal_ros: board %s %dx%d, n_d=%d, radius %.4f m, distance %.4f m",
             rig_.board.type.c_str(), rig_.board.n_x, rig_.board.n_y, rig_.board.n_d,
             rig_.board.radius, rig_.board.distance);
    ROS_INFO("discocal_ros: collecting %d accepted frame(s) per attempt, at most %.0f s",
             rig_.frames.budget, rig_.frames.max_duration_s);

    // The vendored detector reads DISC_EDGE_CHECK from the environment on every
    // call, which is the only hook it offers for its blob appearance gate -- so
    // the rig config has to reach it through setenv. Writing "0" rather than
    // leaving it unset matters: the gate's enable test is `atoi(...) != 0`, and
    // an unset variable has to mean "off" for a stock run. See
    // BoardConfig::edge_check.
    if (setenv("DISC_EDGE_CHECK", rig_.board.edge_check ? "1" : "0", 1) != 0) {
        ROS_WARN("discocal_ros: could not set DISC_EDGE_CHECK; the detector will fall back to "
                 "its own default, which is the gate OFF");
    }
    if (rig_.board.edge_check) {
        ROS_INFO("discocal_ros: blob appearance gate ON (board.edge_check)");
    } else {
        ROS_WARN("discocal_ros: blob appearance gate OFF (board.edge_check=false); the detector "
                 "keeps the %d largest blobs, so static dark objects in view can evict real dots",
                 rig_.board.n_x * rig_.board.n_y);
    }

    // One detector per camera. draw=false: nothing here needs the annotated
    // image, and drawing it costs time on every frame.
    ros::NodeHandle image_nh;
    image_nh.setCallbackQueue(&image_queue_);
    for (std::map<std::string, CameraConfig>::const_iterator it = rig_.cameras.begin();
         it != rig_.cameras.end(); ++it) {
        detectors_[it->first].reset(
            new TargetDetector(rig_.board.n_x, rig_.board.n_y, rig_.board.asymmetric, false));
        sources_.push_back(std::unique_ptr<ImageSource>(
            new ImageSource(image_nh, it->second, &DiscocalNode::onFrame, this, &collecting_)));
    }
    image_spinner_.reset(new ros::AsyncSpinner(
        static_cast<std::uint32_t>(std::max(1, rig_.performance.async_spinner_threads)),
        &image_queue_));
    image_spinner_->start();

    if (rig_.cameras.size() == 2) {
        ROS_INFO("discocal_ros: both-camera goals available (camera: \"both\"); budget is per "
                 "camera");
    }

    // A node that cannot check anything should say so at startup, not after the
    // first two-minute collection. Neither condition is fatal: the session
    // reports the same reasons per goal, and a goal may pass allow_unchecked.
    if (!rig_.check.enabled) {
        ROS_WARN("discocal_ros: check.enabled is false; calibrations will not be verified");
    } else if (!rig_.extrinsic.present) {
        ROS_WARN("discocal_ros: no `extrinsic` in the rig config, so the cross-camera check "
                 "cannot run and every goal will need allow_unchecked");
    }
    for (std::map<std::string, CameraConfig>::const_iterator it = rig_.cameras.begin();
         it != rig_.cameras.end(); ++it) {
        if (it->second.source == "file") {
            ROS_INFO("discocal_ros: %s has source: file -- its intrinsics come from the config "
                     "and are never solved here",
                     it->first.c_str());
        }
    }

    server_.registerGoalCallback(
        [this](ActionServer::GoalHandle handle) { this->onGoal(handle); });
    server_.registerCancelCallback(
        [this](ActionServer::GoalHandle handle) { this->onCancel(handle); });
    server_.start();

    worker_ = std::thread(&DiscocalNode::workerLoop, this);
}

DiscocalNode::~DiscocalNode() {
    // Stop decoding first: no new frame should enter the queue while the
    // worker is being torn down.
    if (image_spinner_) image_spinner_->stop();
    {
        std::lock_guard<std::mutex> lock(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    // The worker may be inside a solve, which cannot be interrupted. Shutdown
    // therefore waits for that solve to finish rather than tearing down a
    // half-written calibration.
    if (worker_.joinable()) worker_.join();
}

void DiscocalNode::onGoal(ActionServer::GoalHandle handle) {
    std::lock_guard<std::mutex> lock(mu_);

    // Accept every goal so its client sees ACTIVE rather than PENDING forever.
    // Only the newest one is kept: one calibration at a time.
    handle.setAccepted("accepted by discocal_ros");

    if (pending_goal_) {
        // A goal was queued but the worker has not started it yet. The latest
        // request wins; end the older one now instead of leaving its client
        // waiting for a result that would never come.
        pending_goal_->setCanceled(
            ActionServer::Result(),
            "superseded by a newer goal before it started");
    }
    pending_goal_.reset(new ActionServer::GoalHandle(handle));

    // A new goal also ends the one in progress, but not here: the worker
    // checks this flag between solve stages, then sets PREEMPTED on the
    // running goal's own handle. The replacement stays queued until then.
    if (active_goal_) preempt_requested_ = true;
    cv_.notify_one();
}

void DiscocalNode::onCancel(ActionServer::GoalHandle handle) {
    std::lock_guard<std::mutex> lock(mu_);

    if (pending_goal_ && *pending_goal_ == handle) {
        // Not started yet: terminate it now and drop it from the queue.
        pending_goal_->setCanceled(ActionServer::Result(), "canceled before it started");
        pending_goal_.reset();
        return;
    }
    if (active_goal_ && *active_goal_ == handle) {
        // Let the worker reach a stage boundary; a Ceres stage in flight runs
        // to completion.
        preempt_requested_ = true;
        cv_.notify_one();
    }
}

void DiscocalNode::succeedGoal(const ActionServer::Result& result, const std::string& text) {
    ActionServer::GoalHandle active;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!active_goal_) return;
        active = *active_goal_;
    }
    active.setSucceeded(result, text);
}

void DiscocalNode::abortGoal(const ActionServer::Result& result, const std::string& text) {
    ActionServer::GoalHandle active;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!active_goal_) return;
        active = *active_goal_;
    }
    active.setAborted(result, text);
}

void DiscocalNode::preemptGoal(const std::string& text) {
    ActionServer::GoalHandle active;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!active_goal_) return;
        active = *active_goal_;
    }
    active.setCanceled(ActionServer::Result(), text);
}

void DiscocalNode::onFrame(void* user, const std::string& camera, std::int64_t stamp_ns,
                           const cv::Mat& bgr) {
    DiscocalNode* self = static_cast<DiscocalNode*>(user);
    // The ImageSource gate normally keeps this from being reached, but an
    // in-flight callback can still land just after collecting_ goes false; the
    // frame belongs to no collection and must not sit in the queue.
    if (!self->collecting_.load()) return;
    std::lock_guard<std::mutex> lock(self->mu_);
    if (self->stop_) return;

    PendingFrame frame;
    frame.camera = camera;
    frame.stamp_ns = stamp_ns;
    frame.bgr = bgr;

    if (self->queue_.size() >= kMaxQueuedFrames) {
        self->queue_.pop_front();  // the oldest frame is the least useful
    }
    self->queue_.push_back(frame);
    self->cv_.notify_one();
}

void DiscocalNode::drainQueue() {
    std::lock_guard<std::mutex> lock(mu_);
    queue_.clear();
}

bool DiscocalNode::popFrame(PendingFrame* frame, int timeout_ms) {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                 [this]() { return stop_ || pending_goal_ || !queue_.empty(); });
    if (queue_.empty()) return false;
    *frame = queue_.front();
    queue_.pop_front();
    return true;
}

bool DiscocalNode::goalPending() {
    std::lock_guard<std::mutex> lock(mu_);
    return pending_goal_ != NULL;
}

std::map<std::string, ::Params> DiscocalNode::knownIntrinsics() const {
    std::map<std::string, ::Params> known = calibrated_;
    for (std::map<std::string, CameraConfig>::const_iterator it = rig_.cameras.begin();
         it != rig_.cameras.end(); ++it) {
        // A calibration done earlier in this process beats the config file.
        if (it->second.source == "file" && it->second.has_intrinsics &&
            known.find(it->first) == known.end()) {
            known[it->first] = it->second.intrinsics;
        }
    }
    return known;
}

void DiscocalNode::publishFeedback(const std::string& state, const CalibrateSession* session,
                                   int budget, bool force_log) {
    ActionServer::Feedback feedback;
    feedback.state = state;
    feedback.frames_seen = static_cast<uint32_t>(std::max(0, frames_seen_));
    feedback.frames_detected = static_cast<uint32_t>(std::max(0, frames_detected_));
    feedback.budget = static_cast<uint32_t>(std::max(0, budget));
    if (session != NULL) {
        const SessionStats& stats = session->stats();
        feedback.frames_accepted = static_cast<uint32_t>(std::max(0, stats.accepted));
        feedback.frames_duplicate = static_cast<uint32_t>(std::max(0, stats.duplicate));
        feedback.frames_unpaired = static_cast<uint32_t>(std::max(0, stats.unpaired));
    }
    feedback.progress =
        budget > 0 ? static_cast<float>(feedback.frames_accepted) / static_cast<float>(budget)
                   : 0.0f;
    if (feedback.progress > 1.0f) feedback.progress = 1.0f;

    // One entry per camera in the goal. Mono goals have exactly one, mirroring
    // the scalar counters above; both-mode goals fill one per camera.
    feedback.cameras_in_goal = 0;
    feedback.cameras.clear();
    if (session != NULL) {
        feedback.cameras_in_goal = 1;
        CameraProgress progress_entry;
        progress_entry.camera = session->camera();
        progress_entry.frames_seen = feedback.frames_seen;
        progress_entry.frames_detected = feedback.frames_detected;
        progress_entry.frames_accepted = feedback.frames_accepted;
        progress_entry.frames_duplicate = feedback.frames_duplicate;
        progress_entry.frames_unpaired = feedback.frames_unpaired;
        progress_entry.budget = feedback.budget;
        progress_entry.progress = feedback.progress;
        feedback.cameras.push_back(progress_entry);
    }

    // Feedback belongs to the goal the worker is running, not to whatever goal
    // actionlib last accepted: a replacement can be queued while this one is
    // still collecting.
    ActionServer::GoalHandle active;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!active_goal_) return;
        active = *active_goal_;
    }
    active.publishFeedback(feedback);

    // Log when the accepted count actually moves, so a duplicate-heavy stretch
    // does not repeat the same line at frame rate.
    const std::string camera = session != NULL ? session->camera() : std::string();
    const int accepted = static_cast<int>(feedback.frames_accepted);
    if (force_log || last_logged_accepted_[camera] != accepted) {
        last_logged_accepted_[camera] = accepted;
        ROS_INFO("discocal_ros: %s: %s: %u/%u accepted (%u detected, %u duplicate, %u unpaired)",
                 state.c_str(), camera.c_str(), feedback.frames_accepted, feedback.budget,
                 feedback.frames_detected, feedback.frames_duplicate, feedback.frames_unpaired);
    }
}

void DiscocalNode::publishRigFeedback(const std::string& state, int budget, bool force_log) {
    ActionServer::Feedback feedback;
    feedback.state = state;
    feedback.cameras_in_goal = 0;

    if (rig_session_ != NULL) {
        const std::vector<std::string>& cameras = rig_session_->cameras();
        feedback.cameras_in_goal = static_cast<std::uint32_t>(cameras.size());
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            const CameraProgress progress = [&]() {
                CameraProgress entry;
                entry.camera = cameras[i];
                const int seen = rig_frames_seen_.count(cameras[i])
                                     ? rig_frames_seen_[cameras[i]]
                                     : 0;
                const int detected = rig_frames_detected_.count(cameras[i])
                                         ? rig_frames_detected_[cameras[i]]
                                         : 0;
                const SessionStats& stats = rig_session_->stats(cameras[i]);
                entry.frames_seen = static_cast<std::uint32_t>(std::max(0, seen));
                entry.frames_detected = static_cast<std::uint32_t>(std::max(0, detected));
                entry.frames_accepted = static_cast<std::uint32_t>(std::max(0, stats.accepted));
                entry.frames_duplicate =
                    static_cast<std::uint32_t>(std::max(0, stats.duplicate));
                entry.frames_unpaired = static_cast<std::uint32_t>(std::max(0, stats.unpaired));
                entry.budget = static_cast<std::uint32_t>(std::max(0, budget));
                entry.progress =
                    budget > 0 ? static_cast<float>(entry.frames_accepted) /
                                     static_cast<float>(budget)
                               : 0.0f;
                if (entry.progress > 1.0f) entry.progress = 1.0f;
                return entry;
            }();
            feedback.cameras.push_back(progress);
        }
        // Legacy scalar feedback mirrors the first camera; the per-camera truth
        // is in cameras[].
        if (!feedback.cameras.empty()) {
            const CameraProgress& first = feedback.cameras[0];
            feedback.frames_seen = first.frames_seen;
            feedback.frames_detected = first.frames_detected;
            feedback.frames_accepted = first.frames_accepted;
            feedback.frames_duplicate = first.frames_duplicate;
            feedback.frames_unpaired = first.frames_unpaired;
            feedback.budget = first.budget;
            feedback.progress = first.progress;
        }
    }

    // Feedback belongs to the goal the worker is running, not to whatever goal
    // actionlib last accepted: a replacement can be queued while this one is
    // still collecting.
    ActionServer::GoalHandle active;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!active_goal_) return;
        active = *active_goal_;
    }
    active.publishFeedback(feedback);

    bool changed = force_log;
    for (std::size_t i = 0; i < feedback.cameras.size(); ++i) {
        const CameraProgress& entry = feedback.cameras[i];
        if (last_logged_accepted_[entry.camera] != static_cast<int>(entry.frames_accepted)) {
            changed = true;
        }
    }
    if (!changed) return;
    for (std::size_t i = 0; i < feedback.cameras.size(); ++i) {
        const CameraProgress& entry = feedback.cameras[i];
        last_logged_accepted_[entry.camera] = static_cast<int>(entry.frames_accepted);
        ROS_INFO("discocal_ros: %s: %s: %u/%u accepted (%u detected, %u duplicate, %u unpaired)",
                 state.c_str(), entry.camera.c_str(), entry.frames_accepted, entry.budget,
                 entry.frames_detected, entry.frames_duplicate, entry.frames_unpaired);
    }
}

std::string DiscocalNode::intrinsicsSummary(const ::Params& params) const {
    std::ostringstream oss;
    oss.precision(6);
    oss << "fx " << params.fx() << " fy " << params.fy() << " cx " << params.cx() << " cy "
        << params.cy() << " skew " << params.skew() << " d1 " << params.d[0] << " d2 "
        << params.d[1];
    return oss.str();
}

void DiscocalNode::fillCameraCalibration(const std::string& camera,
                                         const SessionResult& result,
                                         bool uncertainties_from_mono_stage,
                                         CameraCalibration* out) const {
    out->camera = camera;
    out->success = result.success;
    out->message = result.message;
    out->reprojection_error = result.reprojection_error;
    out->n_frames = static_cast<uint32_t>(std::max(0, result.n_frames));
    out->n_frames_heldout = static_cast<uint32_t>(std::max(0, result.n_frames_heldout));
    out->camera_info_path = result.camera_info_path;
    out->fit_seconds = result.fit_seconds;
    out->uncertainties_from_mono_stage = uncertainties_from_mono_stage;

    out->check_ran = result.check.ran;
    out->check_passed = result.check.passed;
    out->check_pairs = static_cast<uint32_t>(std::max(0, result.check.pairs));
    out->check_rms_px = result.check.rms_px;
    out->check_max_rms_px = result.check.limit_px;
    out->check_worst_pair_rms_px = result.check.worst_pair_rms_px;

    // Zero every intrinsic by default. They are filled in only for a
    // calibration that passed: on a failed check the solve still produced
    // numbers, but they are the numbers the check just rejected, and a consumer
    // that reads this result without looking at `success` would install them.
    // They go in the log and the run's YAML instead.
    out->fx = out->fy = out->cx = out->cy = out->skew = 0.0;
    out->d1 = out->d2 = out->d3 = out->d4 = 0.0;
    out->s_fx = out->s_fy = out->s_cx = out->s_cy = out->s_skew = 0.0;
    out->s_d1 = out->s_d2 = out->s_d3 = out->s_d4 = 0.0;
    if (!result.success) return;

    const ::Params& p = result.params;
    out->fx = p.fx();
    out->fy = p.fy();
    out->cx = p.cx();
    out->cy = p.cy();
    out->skew = p.skew();
    out->d1 = p.d[0];
    out->d2 = p.d[1];
    out->d3 = p.d[2];
    out->d4 = p.d[3];

    out->s_fx = p.s_fx();
    out->s_fy = p.s_fy();
    out->s_cx = p.s_cx();
    out->s_cy = p.s_cy();
    out->s_skew = p.s_skew();
    out->s_d1 = p.s_d[0];
    out->s_d2 = p.s_d[1];
    out->s_d3 = p.s_d[2];
    out->s_d4 = p.s_d[3];
}

void DiscocalNode::fillResult(const std::string& camera, const SessionResult& result,
                              ActionServer::Result* out) const {
    out->success = result.success;
    out->message = result.message;
    out->reprojection_error = result.reprojection_error;
    out->n_frames = static_cast<uint32_t>(std::max(0, result.n_frames));
    out->n_frames_heldout = static_cast<uint32_t>(std::max(0, result.n_frames_heldout));
    out->camera_info_path = result.camera_info_path;
    out->attempts = static_cast<uint32_t>(std::max(0, result.attempts));

    out->check_ran = result.check.ran;
    out->check_passed = result.check.passed;
    out->check_pairs = static_cast<uint32_t>(std::max(0, result.check.pairs));
    out->check_rms_px = result.check.rms_px;
    out->check_max_rms_px = result.check.limit_px;
    out->check_worst_pair_rms_px = result.check.worst_pair_rms_px;

    out->fx = out->fy = out->cx = out->cy = out->skew = 0.0;
    out->d1 = out->d2 = out->d3 = out->d4 = 0.0;
    out->s_fx = out->s_fy = out->s_cx = out->s_cy = out->s_skew = 0.0;
    out->s_d1 = out->s_d2 = out->s_d3 = out->s_d4 = 0.0;
    if (result.success) {
        const ::Params& p = result.params;
        out->fx = p.fx();
        out->fy = p.fy();
        out->cx = p.cx();
        out->cy = p.cy();
        out->skew = p.skew();
        out->d1 = p.d[0];
        out->d2 = p.d[1];
        out->d3 = p.d[2];
        out->d4 = p.d[3];

        out->s_fx = p.s_fx();
        out->s_fy = p.s_fy();
        out->s_cx = p.s_cx();
        out->s_cy = p.s_cy();
        out->s_skew = p.s_skew();
        out->s_d1 = p.s_d[0];
        out->s_d2 = p.s_d[1];
        out->s_d3 = p.s_d[2];
        out->s_d4 = p.s_d[3];
    }

    // Mono goals keep the legacy scalars exactly as before and mirror them in
    // one cameras[] entry. Both-mode goals fill cameras[] per camera and zero
    // the scalars; that fill is done by the rig path, not here.
    out->cameras.clear();
    CameraCalibration camera_result;
    fillCameraCalibration(camera, result, false, &camera_result);
    out->cameras.push_back(camera_result);

    out->joint_refined = false;
    out->joint_fit_seconds = 0.0;
    // Mono runs write straight into output.directory rather than a timestamped
    // run directory; camera_info_path names the file they wrote.
    out->run_directory.clear();
    out->dwecal_path.clear();
    out->dwecal_sha256.clear();
}

void DiscocalNode::fillRigResult(const RigSessionResult& result,
                                 ActionServer::Result* out) const {
    out->success = result.success;
    out->message = result.message;
    out->attempts = static_cast<std::uint32_t>(std::max(0, result.attempts));

    out->joint_refined = result.joint_refined;
    out->joint_fit_seconds = result.joint_fit_seconds;
    out->run_directory = result.run_directory;
    out->dwecal_path = result.dwecal_path;
    out->dwecal_sha256 = result.dwecal_sha256;

    // Legacy intrinsic scalars are always zero in both mode: no single set of
    // numbers describes the rig, and a consumer that ignores `success` or
    // `cameras[]` must not be able to install one camera's values.
    out->fx = out->fy = out->cx = out->cy = out->skew = 0.0;
    out->d1 = out->d2 = out->d3 = out->d4 = 0.0;
    out->s_fx = out->s_fy = out->s_cx = out->s_cy = out->s_skew = 0.0;
    out->s_d1 = out->s_d2 = out->s_d3 = out->s_d4 = 0.0;
    out->camera_info_path.clear();

    out->cameras.clear();
    int total_frames = 0;
    int total_heldout = 0;
    double worst_reprojection = 0.0;
    bool any_check_ran = false;
    bool all_checks_ran = !result.cameras.empty();
    bool all_checks_passed = true;
    const CheckResult* worst_check = NULL;
    for (std::size_t i = 0; i < result.cameras.size(); ++i) {
        const CameraOutcome& camera = result.cameras[i];
        CameraCalibration entry;
        fillCameraCalibration(camera.camera, camera.result,
                              camera.uncertainties_from_mono_stage, &entry);
        out->cameras.push_back(entry);

        total_frames += camera.result.n_frames;
        total_heldout += camera.result.n_frames_heldout;
        worst_reprojection = std::max(worst_reprojection, camera.result.reprojection_error);

        if (!camera.result.check.ran) {
            all_checks_ran = false;
            continue;
        }
        any_check_ran = true;
        if (!camera.result.check.passed) all_checks_passed = false;
        if (worst_check == NULL ||
            camera.result.check.rms_px > worst_check->rms_px) {
            worst_check = &camera.result.check;
        }
    }

    out->n_frames = static_cast<std::uint32_t>(std::max(0, total_frames));
    out->n_frames_heldout = static_cast<std::uint32_t>(std::max(0, total_heldout));
    out->reprojection_error = worst_reprojection;

    // The check scalars describe the worst camera, as documented in the action.
    out->check_ran = all_checks_ran;
    out->check_passed = all_checks_ran && all_checks_passed;
    out->check_pairs = 0;
    out->check_rms_px = 0.0;
    out->check_max_rms_px = 0.0;
    out->check_worst_pair_rms_px = 0.0;
    if (any_check_ran && worst_check != NULL) {
        out->check_pairs = static_cast<std::uint32_t>(std::max(0, worst_check->pairs));
        out->check_rms_px = worst_check->rms_px;
        out->check_max_rms_px = worst_check->limit_px;
        out->check_worst_pair_rms_px = worst_check->worst_pair_rms_px;
    }
}

bool DiscocalNode::stopRequested() {
    std::lock_guard<std::mutex> lock(mu_);
    return stop_ || pending_goal_ || preempt_requested_;
}

bool DiscocalNode::processFrame(const PendingFrame& frame) {
    if (!session_) return false;

    const bool is_goal_camera = frame.camera == session_->camera();
    if (!is_goal_camera && frame.camera != session_->other_camera()) return false;

    const std::map<std::string, std::unique_ptr<TargetDetector> >::iterator detector =
        detectors_.find(frame.camera);
    if (detector == detectors_.end() || !detector->second) return false;

    cv::Mat gray = TargetDetector::preprocessing(frame.bgr, rig_.board.detection_mode);
    if (gray.rows == 0) return false;

    const Target target = detector->second->detect(gray, rig_.board.type);

    BoardFrame board_frame;
    board_frame.camera = frame.camera;
    board_frame.stamp_ns = frame.stamp_ns;
    board_frame.shapes = target.second;

    if (is_goal_camera) {
        ++frames_seen_;
        if (!target.first) return false;
        ++frames_detected_;
        // The first decoded frame of the goal camera is what tells the session
        // the image size the gate normalises by.
        if (!session_->has_image_size()) session_->set_image_size(gray.cols, gray.rows);
        session_->on_detection(board_frame);
        return true;
    }

    // The other camera's frames exist only to pair with. They are not counted in
    // the feedback's frame counters, which describe the collection.
    ++other_seen_;
    if (!target.first) return false;
    ++other_detected_;
    session_->on_other_detection(board_frame);
    return true;
}

bool DiscocalNode::processRigFrame(const PendingFrame& frame) {
    if (!rig_session_) return false;

    const std::map<std::string, std::unique_ptr<TargetDetector> >::iterator detector =
        detectors_.find(frame.camera);
    if (detector == detectors_.end() || !detector->second) return false;

    cv::Mat gray = TargetDetector::preprocessing(frame.bgr, rig_.board.detection_mode);
    if (gray.rows == 0) return false;

    const Target target = detector->second->detect(gray, rig_.board.type);

    ++rig_frames_seen_[frame.camera];
    if (!target.first) return false;
    ++rig_frames_detected_[frame.camera];

    BoardFrame board_frame;
    board_frame.camera = frame.camera;
    board_frame.stamp_ns = frame.stamp_ns;
    board_frame.shapes = target.second;

    // The first decoded frame of each camera tells its session the image size
    // the diversity gate normalises by and the calibrator scales matrices with.
    if (!rig_session_->has_image_size(frame.camera)) {
        rig_session_->set_image_size(frame.camera, gray.cols, gray.rows);
    }
    // RigSession routes the frame to its owner and, as "the other camera", to
    // the partner's pairing history.
    rig_session_->on_detection(board_frame);
    return true;
}

void DiscocalNode::runGoal(const boost::shared_ptr<const ActionServer::Goal>& goal) {
    const std::string camera = goal->camera;
    const bool allow_unchecked = goal->allow_unchecked;

    ActionServer::Result result;

    if (both_mode(camera)) {
        if (rig_.cameras.size() != 2) {
            std::ostringstream oss;
            oss << "camera: \"" << camera << "\" calibrates both cameras, but the rig config "
                << "declares " << rig_.cameras.size() << " camera(s); exactly two are required";
            result.message = oss.str();
            ROS_ERROR("discocal_ros: %s", result.message.c_str());
            abortGoal(result, "");
            return;
        }
        std::vector<std::string> cameras;
        for (std::map<std::string, CameraConfig>::const_iterator it = rig_.cameras.begin();
             it != rig_.cameras.end(); ++it) {
            if (it->second.source != "calibrate") {
                result.message = "camera: \"" + camera + "\" needs both cameras to have source: "
                                 "calibrate, but \"" + it->first + "\" is source: " +
                                 it->second.source;
                ROS_ERROR("discocal_ros: %s", result.message.c_str());
                abortGoal(result, "");
                return;
            }
            cameras.push_back(it->first);
        }
        runRigGoal(goal, cameras);
        return;
    }

    const CameraConfig* camera_config = rig_.camera(camera);
    if (camera_config == NULL) {
        std::ostringstream oss;
        oss << "no camera \"" << camera << "\" in the rig config; it declares:";
        for (std::map<std::string, CameraConfig>::const_iterator it = rig_.cameras.begin();
             it != rig_.cameras.end(); ++it) {
            oss << " " << it->first;
        }
        result.message = oss.str();
        ROS_ERROR("discocal_ros: %s", result.message.c_str());
        abortGoal(result, "");
        return;
    }
    if (camera_config->source != "calibrate") {
        result.message = "\"" + camera +
                         "\" is declared source: file in the rig config, so this node will not "
                         "solve it; change it to source: calibrate to collect for it";
        ROS_ERROR("discocal_ros: %s", result.message.c_str());
        abortGoal(result, "");
        return;
    }

    const int effective_budget = goal->budget > 0 ? static_cast<int>(goal->budget)
                                                  : rig_.frames.budget;
    const int max_attempts = 1 + std::max(0, rig_.check.retries);

    // One session for the whole goal, reset between attempts: reset() keeps the
    // other camera's frame history, which is what lets a retry's held-out frames
    // pair at all.
    session_.reset(new CalibrateSession(rig_, camera, effective_budget, knownIntrinsics()));
    if (!session_->check_configured()) {
        ROS_WARN("discocal_ros: the cross-camera check cannot run for %s: %s", camera.c_str(),
                 session_->check_unavailable_reason().c_str());
    }

    SessionResult session_result;
    bool have_result = false;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        frames_seen_ = 0;
        frames_detected_ = 0;
        other_seen_ = 0;
        other_detected_ = 0;
        last_logged_accepted_.clear();
        if (attempt > 1) session_->reset();

        drainQueue();
        collecting_ = true;
        publishFeedback(attempt > 1 ? kStateRetrying : kStateCollecting, session_.get(),
                        effective_budget, true);

        const ros::WallTime start = ros::WallTime::now();
        const ros::WallDuration limit(rig_.frames.max_duration_s);
        bool timed_out = false;

        while (!session_->budget_reached()) {
            if (preempt_requested_ || !ros::ok()) {
                ROS_WARN("discocal_ros: stopping the collection for %s", camera.c_str());
                collecting_ = false;
                session_.reset();
                preemptGoal("preempted");
                return;
            }

            PendingFrame frame;
            if (!popFrame(&frame, kWorkerTickMs)) {
                // Nothing arrived this tick. A new goal would have woken us.
                if (goalPending()) break;
                if (ros::WallTime::now() - start > limit) {
                    timed_out = true;
                    break;
                }
                continue;
            }

            processFrame(frame);
            publishFeedback(kStateCollecting, session_.get(), effective_budget, false);

            if (ros::WallTime::now() - start > limit) {
                timed_out = true;
                break;
            }
        }
        collecting_ = false;

        if (goalPending()) {
            ROS_WARN("discocal_ros: a new goal arrived while collecting for %s; abandoning it",
                     camera.c_str());
            session_.reset();
            preemptGoal("preempted");
            return;
        }

        // `budget` is a collection target, not a minimum. Running out of time
        // with fewer frames is not by itself a failure: the solve below decides,
        // and it only refuses when the accepted count is below the calibrator's
        // own floor. A feed that stops early is a normal way to calibrate with
        // what it delivered.
        if (timed_out) {
            std::ostringstream oss;
            // fixed, or precision(0) prints 120 as "1e+02" and the message reads
            // like a bug.
            oss << std::fixed;
            oss.precision(0);
            oss << "collection window of " << rig_.frames.max_duration_s
                << " s ended with " << session_->stats().accepted << " of "
                << effective_budget << " accepted frame(s); the board was detected in "
                << frames_detected_ << " of " << frames_seen_
                << " decoded frame(s), solving with what was collected";
            ROS_WARN("discocal_ros: %s: %s", camera.c_str(), oss.str().c_str());
        }

        publishFeedback(kStateCalibrating, session_.get(), effective_budget, true);
        ROS_INFO("discocal_ros: %s: solving from %d accepted frame(s) (attempt %d of %d)",
                 camera.c_str(), session_->stats().accepted, attempt, max_attempts);

        // The slow part, and the one that cannot be interrupted: the fit and the
        // held-out solve run to completion.
        session_result = session_->finalize(allow_unchecked);
        have_result = true;

        // Reported after finalize, because that is where the session recomputes
        // the true paired count -- during collection it can only say whether the
        // other camera's frame for a stamp has shown up yet, and in a feed that
        // publishes one camera and then the other that is always "not yet".
        ROS_INFO("discocal_ros: %s: other camera (%s): %d of %d decoded frame(s) detected; "
                 "%d of %d accepted frame(s) had a simultaneous frame there",
                 camera.c_str(), session_->other_camera().c_str(), other_detected_, other_seen_,
                 session_->stats().accepted - session_->stats().unpaired,
                 session_->stats().accepted);

        if (session_result.success) break;

        ROS_WARN("discocal_ros: %s: attempt %d failed: %s", camera.c_str(), attempt,
                 session_result.message.c_str());
        // Report what the rejected solve produced. The action result deliberately
        // carries no intrinsics on a failure (see fillResult), and tuning
        // check.max_rms_px needs exactly these numbers.
        if (session_result.params.fx() > 0.0) {
            ROS_WARN("discocal_ros: %s: the rejected solve gave %s", camera.c_str(),
                     intrinsicsSummary(session_result.params).c_str());
        }

        // A retry only re-collects; it cannot cure a frame count below the
        // solver's floor, and on a feed that has run dry it would just burn a
        // second collection window. Stop here and report the count.
        if (session_->stats().accepted < session_->min_fit_frames()) {
            ROS_ERROR("discocal_ros: %s: only %d accepted frame(s); the solver needs at least %d, "
                      "so retrying cannot help",
                      camera.c_str(), session_->stats().accepted, session_->min_fit_frames());
            break;
        }
    }

    if (!have_result) {
        result.message = "no calibration attempt ran";
        abortGoal(result, "");
        return;
    }

    // Remember a good result so that calibrating the *other* camera afterwards
    // can check against it rather than against the config file.
    if (session_result.success) calibrated_[camera] = session_result.params;

    fillResult(camera, session_result, &result);
    const bool succeeded = session_result.success;
    publishFeedback(succeeded ? kStateDone : kStateFailed, session_.get(), effective_budget, true);

    // A goal that arrived while the solve was in flight preempts this one. It is
    // checked after fillResult so the result still describes what ran.
    const bool cancelled = preempt_requested_ || goalPending();
    session_.reset();

    if (cancelled) {
        ROS_WARN("discocal_ros: %s: preempted", camera.c_str());
        preemptGoal("preempted");
    } else if (succeeded) {
        ROS_INFO("discocal_ros: %s: %s", camera.c_str(), result.message.c_str());
        succeedGoal(result, "");
    } else {
        ROS_ERROR("discocal_ros: %s: %s", camera.c_str(), result.message.c_str());
        abortGoal(result, "");
    }
}

void DiscocalNode::runRigGoal(const boost::shared_ptr<const ActionServer::Goal>& goal,
                              const std::vector<std::string>& cameras) {
    const bool allow_unchecked = goal->allow_unchecked;
    const int effective_budget =
        goal->budget > 0 ? static_cast<int>(goal->budget) : rig_.frames.budget;
    const int max_attempts = 1 + std::max(0, rig_.check.retries);

    ActionServer::Result result;
    performance_.begin(goal->camera, effective_budget);

    // Neither camera's result exists at the start, so the known-intrinsics map
    // is deliberately empty: RigSession supplies each partner's final params
    // once both mono solves have run.
    rig_session_.reset(
        new RigSession(rig_, cameras, effective_budget, std::map<std::string, ::Params>()));
    if (!rig_.extrinsic.present) {
        ROS_WARN("discocal_ros: both-camera goals need allow_unchecked without an `extrinsic` in "
                 "the rig config: neither camera can be checked");
    }

    RigSessionResult rig_result;
    bool have_result = false;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        rig_frames_seen_.clear();
        rig_frames_detected_.clear();
        last_logged_accepted_.clear();
        if (attempt > 1) rig_session_->reset();

        drainQueue();
        StageClock collection_clock;
        collecting_ = true;
        publishRigFeedback(attempt > 1 ? kStateRetrying : kStateCollecting, effective_budget, true);

        const ros::WallTime start = ros::WallTime::now();
        const ros::WallDuration limit(rig_.frames.max_duration_s);
        bool timed_out = false;

        while (!rig_session_->budget_reached()) {
            if (preempt_requested_ || !ros::ok()) {
                ROS_WARN("discocal_ros: stopping the both-camera collection");
                collecting_ = false;
                rig_session_.reset();
                preemptGoal("preempted");
                return;
            }

            PendingFrame frame;
            if (!popFrame(&frame, kWorkerTickMs)) {
                if (goalPending()) break;
                if (ros::WallTime::now() - start > limit) {
                    timed_out = true;
                    break;
                }
                continue;
            }

            processRigFrame(frame);
            publishRigFeedback(kStateCollecting, effective_budget, false);

            if (ros::WallTime::now() - start > limit) {
                timed_out = true;
                break;
            }
        }
        collecting_ = false;

        // Counters are summed across attempts by the report, because a retry
        // resets the session's own stats and the goal's CPU/wall totals still
        // include the failed try.
        {
            CollectionCounters counters;
            for (std::size_t i = 0; i < cameras.size(); ++i) {
                const std::string& camera = cameras[i];
                counters.frames_decoded +=
                    rig_frames_seen_.count(camera) ? rig_frames_seen_[camera] : 0;
                counters.frames_detected +=
                    rig_frames_detected_.count(camera) ? rig_frames_detected_[camera] : 0;
                const SessionStats& stats = rig_session_->stats(camera);
                counters.frames_accepted += stats.accepted;
                counters.frames_duplicate += stats.duplicate;
                counters.frames_unpaired += stats.unpaired;
            }
            performance_.add_collection(counters, collection_clock.stop());
        }

        if (goalPending()) {
            ROS_WARN("discocal_ros: a new goal arrived while collecting both cameras; abandoning "
                     "this one");
            rig_session_.reset();
            preemptGoal("preempted");
            return;
        }

        // `budget` is per-camera collection target, not a minimum. A window
        // that ends short is not a failure by itself: solving goes ahead with
        // whatever was collected, and each camera only fails at solve time if
        // it has fewer accepted frames than the calibrator can use.
        if (timed_out) {
            std::ostringstream oss;
            oss << std::fixed;
            oss.precision(0);
            oss << "collection window of " << rig_.frames.max_duration_s << " s ended; collecting";
            for (std::size_t i = 0; i < cameras.size(); ++i) {
                oss << " " << cameras[i] << "=" << rig_session_->stats(cameras[i]).accepted << "/"
                    << effective_budget;
            }
            oss << " accepted frame(s), solving with what was collected";
            ROS_WARN("discocal_ros: %s", oss.str().c_str());
        }

        publishRigFeedback(kStateCalibrating, effective_budget, true);
        std::ostringstream starting;
        starting << "solving";
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            starting << " " << cameras[i] << "=" << rig_session_->stats(cameras[i]).accepted;
        }
        starting << " accepted frame(s) (attempt " << attempt << " of " << max_attempts << ")";
        ROS_INFO("discocal_ros: %s", starting.str().c_str());

        // The slow part. The callback is polled between solve stages; a Ceres
        // stage in flight runs to completion.
        rig_result = rig_session_->solve(
            allow_unchecked, [this]() { return this->stopRequested(); });
        have_result = true;

        if (preempt_requested_ || goalPending()) break;
        if (rig_result.success) break;

        ROS_WARN("discocal_ros: both-camera attempt %d failed: %s", attempt,
                 rig_result.message.c_str());

        // As in the mono path: a retry re-collects from zero and cannot cure a
        // camera that never reached the solver's floor.
        bool starved = false;
        for (std::size_t i = 0; i < cameras.size(); ++i) {
            if (rig_session_->stats(cameras[i]).accepted <
                rig_session_->min_fit_frames(cameras[i])) {
                ROS_ERROR("discocal_ros: %s: only %d accepted frame(s); the solver needs at least "
                          "%d, so retrying cannot help",
                          cameras[i].c_str(), rig_session_->stats(cameras[i]).accepted,
                          rig_session_->min_fit_frames(cameras[i]));
                starved = true;
            }
        }
        if (starved) break;
    }

    if (!have_result) {
        result.message = "no both-camera calibration attempt ran";
        abortGoal(result, "");
        return;
    }

    // Remember good results so a later mono goal can check against them.
    if (rig_result.success) {
        for (std::size_t i = 0; i < rig_result.cameras.size(); ++i) {
            if (rig_result.cameras[i].result.success) {
                calibrated_[rig_result.cameras[i].camera] = rig_result.cameras[i].result.params;
            }
        }
    }

    fillRigResult(rig_result, &result);
    const bool succeeded = rig_result.success;
    publishRigFeedback(succeeded ? kStateDone : kStateFailed, effective_budget, true);

    performance_.set_solve_timings(rig_result.timings);
    performance_.set_attempts(rig_result.attempts);
    for (std::size_t i = 0; i < rig_result.warnings.size(); ++i) {
        performance_.add_warning(rig_result.warnings[i]);
    }
    DwecalTiming dwecal;
    if (!rig_result.dwecal_path.empty()) {
        dwecal.width = rig_.dwecal.output_width;
        dwecal.height = rig_.dwecal.output_height;
        dwecal.sha256 = rig_result.dwecal_sha256;
        dwecal.timing = rig_result.timings.dwecal;
        struct stat info;
        if (stat(rig_result.dwecal_path.c_str(), &info) == 0 && info.st_size > 0) {
            dwecal.bytes = static_cast<std::uint64_t>(info.st_size);
        }
    }
    performance_.set_dwecal(dwecal);
    performance_.finish(succeeded, result.message);
    if (!rig_result.run_directory.empty()) {
        std::string performance_error;
        if (!performance_.write(rig_result.run_directory + "/performance.yaml",
                                &performance_error)) {
            ROS_ERROR("discocal_ros: could not write performance.yaml: %s",
                      performance_error.c_str());
        }
    }

    const bool cancelled = preempt_requested_ || goalPending();
    rig_session_.reset();

    if (cancelled) {
        ROS_WARN("discocal_ros: both-camera goal preempted");
        preemptGoal("preempted");
    } else if (succeeded) {
        ROS_INFO("discocal_ros: %s", result.message.c_str());
        succeedGoal(result, "");
    } else {
        ROS_ERROR("discocal_ros: %s", result.message.c_str());
        abortGoal(result, "");
    }
}

void DiscocalNode::workerLoop() {
    // OpenMP's nthreads-var is per-thread: setting it in the constructor would
    // only limit the main thread, leaving the detector's parallel regions here
    // at the library default (one per core). The env var cannot help either,
    // because libgomp reads OMP_NUM_THREADS when the library is loaded, before
    // main(). Apply the configured limit on the thread that detects.
    TargetDetector::set_num_threads(rig_.performance.detector_threads);
    ROS_INFO("discocal_ros: OpenMP max threads on the detection worker: %d (config %d)",
             omp_get_max_threads(), rig_.performance.detector_threads);

    // PRIO_PROCESS with who=0 is the calling thread on Linux, so this lowers
    // only the worker's priority; the action server and image spinner keep
    // theirs and stay responsive while a solve runs.
    if (rig_.performance.nice != 0) {
        if (setpriority(PRIO_PROCESS, 0, rig_.performance.nice) != 0) {
            ROS_WARN("discocal_ros: could not set the worker's nice value to %d: %s",
                     rig_.performance.nice, std::strerror(errno));
        } else {
            ROS_INFO("discocal_ros: worker nice value set to %d", rig_.performance.nice);
        }
    }

    while (true) {
        boost::shared_ptr<ActionServer::GoalHandle> goal_handle;
        {
            std::unique_lock<std::mutex> lock(mu_);
            cv_.wait_for(lock, std::chrono::milliseconds(kWorkerTickMs),
                         [this]() { return stop_ || pending_goal_ || !queue_.empty(); });
            if (stop_) return;
            if (pending_goal_) {
                goal_handle = pending_goal_;
                pending_goal_.reset();
                // The callbacks can now tell a cancel of the running goal from
                // a cancel of a queued one. Feedback and the terminal transition
                // go to this handle, never to whatever arrived later.
                active_goal_ = goal_handle;
                preempt_requested_ = false;
            } else {
                // Nothing is being collected, so queued frames have no consumer.
                queue_.clear();
            }
        }

        if (goal_handle) {
            runGoal(goal_handle->getGoal());
            std::lock_guard<std::mutex> lock(mu_);
            active_goal_.reset();
        }
    }
}

}  // namespace discocal_ros

int main(int argc, char** argv) {
    ros::init(argc, argv, "discocal_ros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::unique_ptr<discocal_ros::DiscocalNode> node;
    try {
        node.reset(new discocal_ros::DiscocalNode(nh, pnh));
    } catch (const std::exception& e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }

    ros::spin();
    return 0;
}
