#include "discocal_ros/calibrate_session.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "discocal_ros/camera_info_writer.h"
#include "discocal_ros/external_points.h"
#include "discocal_ros/known_pose_solver.h"

// The vendored calibrator and its Lie-algebra helpers. Both headers do
// `using namespace std;` at global scope, so they stay in this translation unit.
#include "CCalibrator.h"
#include "CLieAlgebra.h"

namespace discocal_ros {

namespace {

// The calibrator's own projection modes: 0 for a circle board, 2 for a square
// one. See Calibrator::calibrate's `mode` and mono.hpp's driver.
int calibration_mode(const std::string& board_type) {
    return board_type == "circle" ? 0 : 2;
}

std::string attempt_directory(const std::string& output_dir, const std::string& camera, int attempt) {
    std::ostringstream oss;
    oss << output_dir << "/discocal_raw/" << camera << "/attempt_" << attempt;
    return oss.str();
}

std::string format_double(double value) {
    std::ostringstream oss;
    oss.precision(6);
    oss << value;
    return oss.str();
}

}  // namespace

CalibrateSession::CalibrateSession(const RigConfig& rig, const std::string& camera, int budget,
                                  const std::map<std::string, ::Params>& known_intrinsics)
    : rig_(rig),
      camera_(camera),
      budget_(budget > 0 ? budget : rig.frames.budget),
      gate_(rig.frames.diversity_gate, rig.frames.diversity_threshold) {
    other_camera_ = rig.other_camera(camera);
    output_directory_ = rig.output.directory;

    // Everything the check needs, and a reason naming the first thing missing.
    // The other camera's intrinsics are tracked separately (have_other_params_):
    // in both-camera mode they do not exist until both mono solves have run.
    if (!rig.check.enabled) {
        check_unavailable_reason_ = "check.enabled is false";
    } else if (!rig.extrinsic.present) {
        check_unavailable_reason_ = "the rig config has no `extrinsic`";
    } else if (other_camera_.empty()) {
        check_unavailable_reason_ = "the rig does not declare exactly two cameras";
    } else if (rig.extrinsic.parent != camera_ && rig.extrinsic.child != camera_) {
        check_unavailable_reason_ = "the extrinsic is " + rig.extrinsic.parent + " -> " +
                                    rig.extrinsic.child + ", which does not involve \"" + camera_ +
                                    "\"";
    } else {
        check_configured_ = true;

        // The config gives p_parent = R * p_child + t. Oriented so that it maps
        // this camera's frame into the other camera's, and written out rather
        // than inverted so the direction is legible at the point of use.
        const Eigen::Matrix3d R = rig_.extrinsic.R;
        const Eigen::Vector3d t = rig_.extrinsic.t;
        Eigen::Matrix3d R_out;
        Eigen::Vector3d t_out;
        if (camera_ == rig_.extrinsic.parent) {
            R_out = R.transpose();
            t_out = -R.transpose() * t;
        } else {
            R_out = R;
            t_out = t;
        }
        T_own_to_other_ = Eigen::Matrix4d::Identity();
        T_own_to_other_.topLeftCorner<3, 3>() = R_out;
        T_own_to_other_.topRightCorner<3, 1>() = t_out;
        have_other_transform_ = true;

        const std::map<std::string, ::Params>::const_iterator other =
            known_intrinsics.find(other_camera_);
        if (other != known_intrinsics.end()) {
            other_params_ = other->second;
            have_other_params_ = true;
        }
        update_check_unavailable_reason();
    }
}

void CalibrateSession::update_check_unavailable_reason() {
    if (!check_configured_) return;  // the constructor already named the config problem
    if (have_other_params_) {
        check_unavailable_reason_.clear();
    } else {
        check_unavailable_reason_ = "the intrinsics of \"" + other_camera_ + "\" are not known";
    }
}

void CalibrateSession::set_other_params(const ::Params& params) {
    other_params_ = params;
    have_other_params_ = true;
    update_check_unavailable_reason();
}

void CalibrateSession::set_output_directory(const std::string& directory) {
    output_directory_ = directory;
}

void CalibrateSession::set_image_size(int width, int height) {
    width_ = width;
    height_ = height;
    gate_.set_image_size(width, height);
}

bool CalibrateSession::on_detection(const BoardFrame& frame) {
    ++stats_.detected;

    if (!has_image_size()) {
        // Only reachable if the caller feeds a detection before telling the
        // session the image size, which would also make the gate's thresholds
        // meaningless.
        last_decision_ = "no image size has been set; detection ignored";
        return budget_reached();
    }

    // External mode: a frame without a usable pose is not collectable at all.
    // This gate runs before the diversity gate so an unpaired detection neither
    // consumes budget nor pollutes the accepted signatures.
    if (rig_.solve.pose_source == "external") {
        const std::size_t expected =
            static_cast<std::size_t>(rig_.board.n_x) * static_cast<std::size_t>(rig_.board.n_y);

        if (!frame.external.present) {
            ++stats_.pose_missing;
            last_decision_ = "no external pose for this stamp";
            if (!frame.external.error.empty()) last_decision_ += ": " + frame.external.error;
            return budget_reached();
        }

        const CameraConfig* camera = rig_.camera(camera_);
        const std::string expected_frame =
            (camera != NULL && !camera->point_pose_frame_id.empty())
                ? camera->point_pose_frame_id
                : rig_.external.expected_frame_id;

        std::string invalid;
        std::ostringstream oss;
        if (frame.shapes.size() != expected) {
            oss << "detector returned " << frame.shapes.size() << " dot(s), the board has "
                << expected;
            invalid = oss.str();
        } else if (frame.external.points.size() != expected) {
            oss << "external pose has " << frame.external.points.size() << " point(s), the board has "
                << expected;
            invalid = oss.str();
        } else if (!expected_frame.empty() && frame.external.frame_id != expected_frame) {
            invalid = "external pose is in frame \"" + frame.external.frame_id +
                      "\" but the expected frame is \"" + expected_frame + "\"";
        } else {
            for (std::size_t i = 0; i < frame.external.points.size(); ++i) {
                const Eigen::Vector3d& p = frame.external.points[i];
                if (!p.allFinite() || !(p.z() > 0.0)) {
                    oss << "external point " << i
                        << " is not a finite position in front of the camera";
                    invalid = oss.str();
                    break;
                }
            }
        }

        if (!invalid.empty()) {
            ++stats_.pose_invalid;
            last_decision_ = invalid;
            return budget_reached();
        }
    }

    double nearest = -1.0;
    const GateDecision decision = gate_.consider(frame.shapes, &nearest);

    if (decision != GateDecision::kAccepted) {
        ++stats_.duplicate;
        std::ostringstream oss;
        if (decision == GateDecision::kDegenerate) {
            oss << "pose signature undefined for " << frame.shapes.size()
                << " dots; rejected";
        } else {
            oss << "duplicate pose (nearest accepted signature " << format_double(nearest)
                << " < " << format_double(rig_.frames.diversity_threshold) << ")";
        }
        last_decision_ = oss.str();
        return budget_reached();
    }

    accepted_.push_back(frame);
    ++stats_.accepted;

    // Provisional: whether the other camera's frame for this instant has shown
    // up yet. finalize() recomputes the true count, so this can only fall.
    if (other_.find(frame.stamp_ns) == other_.end()) ++stats_.unpaired;

    std::ostringstream oss;
    oss << "accepted " << stats_.accepted << "/" << budget_;
    if (nearest >= 0.0) oss << " (nearest accepted signature " << format_double(nearest) << ")";
    last_decision_ = oss.str();

    return budget_reached();
}

void CalibrateSession::on_other_detection(const BoardFrame& frame) {
    // First detection for a stamp wins. Keeping the history's stamps unique is
    // what lets the pruning below erase by insertion order without ever dropping
    // an entry a later accepted frame still needs.
    if (other_.find(frame.stamp_ns) != other_.end()) return;

    other_[frame.stamp_ns] = frame;
    other_order_.push_back(frame.stamp_ns);

    // Bounded: an accepted frame and the other camera's frame for the same
    // instant arrive within milliseconds of each other, so a few seconds of
    // history is far more than enough.
    while (other_order_.size() > kOtherHistory) {
        other_.erase(other_order_.front());
        other_order_.pop_front();
    }
}

void CalibrateSession::reset() {
    accepted_.clear();
    gate_.reset();
    stats_ = SessionStats();
    last_decision_.clear();
    // `other_` is deliberately left alone: it is keyed by stamp and a retry
    // collects forward in time, so nothing in it can pair with the new frames.
}

int CalibrateSession::min_fit_frames() const {
    // Calibrator::cal_initial_params needs at least 5 scenes; the library's own
    // drivers additionally require 4 + n_d.
    return std::max(5, 4 + rig_.board.n_d);
}

std::vector<int> spread_indices(int count, int n_holdout) {
    std::vector<int> indices;
    if (count <= 0 || n_holdout <= 0) return indices;
    for (int i = 0; i < n_holdout; ++i) {
        int position = static_cast<int>((static_cast<double>(i) + 0.5) * count / n_holdout);
        if (position >= count) position = count - 1;
        indices.push_back(position);
    }
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    return indices;
}

SolveOutcome CalibrateSession::solve(const std::vector<std::int64_t>* forced_holdout,
                                     int attempt) {
    SolveOutcome out;
    const BoardConfig& board = rig_.board;
    const int mode = calibration_mode(board.type);
    const int min_fit = min_fit_frames();
    const int n_accepted = static_cast<int>(accepted_.size());

    if (!has_image_size()) {
        out.message = "no frames were decoded, so the image size is unknown";
        return out;
    }
    if (n_accepted < min_fit) {
        std::ostringstream oss;
        oss << "only " << n_accepted << " accepted frame(s) were collected; a " << board.type
            << " board with n_d=" << board.n_d << " cannot be solved from fewer than " << min_fit;
        out.message = oss.str();
        return out;
    }

    // The true unpaired count, now that every accepted frame is known. During
    // collection this can only say whether the other camera's frame for a stamp
    // had shown up yet, which in a feed that publishes one camera and then the
    // other is always "not yet".
    {
        int unpaired = 0;
        for (int i = 0; i < n_accepted; ++i) {
            if (other_.find(accepted_[i].stamp_ns) == other_.end()) ++unpaired;
        }
        stats_.unpaired = unpaired;
    }

    // --- holdout split ------------------------------------------------------
    // A forced list comes from the both-camera orchestrator and is shared by
    // both cameras; the fraction is then ignored. Without one, the mono path
    // selects exactly as it always has: only when the check is configured and
    // the other camera's intrinsics are already known.
    std::vector<int> holdout_positions;
    if (forced_holdout != NULL) {
        std::vector<std::int64_t> stamps = *forced_holdout;
        std::sort(stamps.begin(), stamps.end());
        for (int i = 0; i < n_accepted; ++i) {
            if (std::binary_search(stamps.begin(), stamps.end(), accepted_[i].stamp_ns)) {
                holdout_positions.push_back(i);
            }
        }
        // Never hold out so many that the fit drops below the library's floor.
        // The orchestrator already sizes the list for both cameras; this is the
        // per-camera guard for a camera that accepted fewer frames.
        const int max_holdout = std::max(0, n_accepted - min_fit);
        if (static_cast<int>(holdout_positions.size()) > max_holdout) {
            const std::vector<int> picked =
                spread_indices(static_cast<int>(holdout_positions.size()), max_holdout);
            std::vector<int> trimmed;
            trimmed.reserve(picked.size());
            for (std::size_t i = 0; i < picked.size(); ++i) {
                trimmed.push_back(holdout_positions[picked[i]]);
            }
            holdout_positions.swap(trimmed);
        }
        if (holdout_positions.empty()) {
            out.holdout_reason = "none of the shared holdout stamps was accepted by this camera";
        }
    } else if (check_configured_ && have_other_params_) {
        // Frames the other camera also saw are preferred for the holdout,
        // because only those can be scored; the rest of the accepted frames
        // still go into the fit.
        std::vector<int> candidates;
        for (int i = 0; i < n_accepted; ++i) {
            if (other_.find(accepted_[i].stamp_ns) != other_.end()) candidates.push_back(i);
        }
        if (candidates.empty()) {
            for (int i = 0; i < n_accepted; ++i) candidates.push_back(i);
        }

        const int n_candidates = static_cast<int>(candidates.size());
        int n_holdout =
            static_cast<int>(std::lround(rig_.frames.holdout_fraction * n_candidates));
        n_holdout = std::min(n_holdout, n_candidates);
        // Never hold so many out that the fit drops below the library's floor.
        n_holdout = std::min(n_holdout, n_accepted - min_fit);

        if (n_holdout >= 1) {
            const std::vector<int> picked = spread_indices(n_candidates, n_holdout);
            for (std::size_t i = 0; i < picked.size(); ++i) {
                holdout_positions.push_back(candidates[picked[i]]);
            }
            std::sort(holdout_positions.begin(), holdout_positions.end());
        } else if (rig_.frames.holdout_fraction <= 0.0) {
            out.holdout_reason = "frames.holdout_fraction is 0, so no frame is held out to score";
        } else {
            out.holdout_reason = "too few accepted frames to hold any out without starving the solve";
        }
    } else {
        out.holdout_reason = check_unavailable_reason_;
    }

    std::vector<int> fit_positions;
    for (int i = 0; i < n_accepted; ++i) {
        if (!std::binary_search(holdout_positions.begin(), holdout_positions.end(), i)) {
            fit_positions.push_back(i);
        }
    }

    // The external path shares every decision up to here -- which frames were
    // accepted and which are held out -- and differs only in where the board
    // pose comes from.
    if (rig_.solve.pose_source == "external") {
        return solve_external(fit_positions, holdout_positions, attempt);
    }

    // --- solve --------------------------------------------------------------
    // The library opens <dir>/intrinsic_parameters.yaml unconditionally inside
    // calibrate(), so it needs a directory that exists. It is left there as a
    // diagnostic: it carries the uncertainties this node only reports through
    // the action.
    out.raw_output_directory = attempt_directory(output_directory_, camera_, attempt);
    std::string error;
    if (!ensure_directory(out.raw_output_directory, &error)) {
        out.message = "cannot prepare the output directory: " + error;
        return out;
    }

    const std::chrono::steady_clock::time_point fit_start = std::chrono::steady_clock::now();
    Calibrator calibrator(board.n_x, board.n_y, board.asymmetric, board.n_d, board.radius,
                          board.distance, static_cast<int>(fit_positions.size()),
                          out.raw_output_directory + "/");
    calibrator.set_image_size(width_, height_);
    for (std::size_t i = 0; i < fit_positions.size(); ++i) {
        calibrator.inputTarget(accepted_[fit_positions[i]].shapes);
    }

    // save_jacob = false. It does not skip the uncertainty block -- that is
    // computed unconditionally in batch_optimize -- it only skips dumping
    // jacob.txt and measurements.txt, which nothing here reads.
    const ::Params params = calibrator.calibrate(mode, false);
    out.fit_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - fit_start).count();
    out.params = params;
    out.n_frames = static_cast<int>(fit_positions.size());
    out.n_frames_heldout = static_cast<int>(holdout_positions.size());

    // A negative focal length is meaningless, and when its initial estimate
    // fails the calibrator silently falls back to width/2, height/2 -- so catch
    // the degenerate cases here rather than reporting them as a calibration.
    if (!(params.fx() > 0.0) || !(params.fy() > 0.0)) {
        out.message = "the calibrator returned a degenerate solution (fx or fy is not positive)";
        return out;
    }
    out.ok = true;

    // --- the library's own reprojection error -------------------------------
    // get_extrinsic() after calibrate() hands back the optimised board poses in
    // the order they were input. That holds because calibrate() samples with
    // sorted_random_sampling(num_scene, num_scene), which draws every index and
    // then sorts -- i.e. the identity permutation.
    const std::vector<se3> fit_poses = calibrator.get_extrinsic();
    out.fit_stamps.reserve(fit_positions.size());
    out.fit_shapes.reserve(fit_positions.size());
    for (std::size_t i = 0; i < fit_positions.size(); ++i) {
        out.fit_stamps.push_back(accepted_[fit_positions[i]].stamp_ns);
        out.fit_shapes.push_back(accepted_[fit_positions[i]].shapes);
    }
    const std::size_t n_poses = std::min(fit_positions.size(), fit_poses.size());
    out.fit_poses.reserve(n_poses);
    for (std::size_t i = 0; i < n_poses; ++i) {
        out.fit_poses.push_back(LieAlgebra::to_SE3(fit_poses[i]));
    }
    {
        const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
        const std::vector<std::vector<::Shape> > detections(out.fit_shapes.begin(),
                                                            out.fit_shapes.begin() + n_poses);
        const BoardProjector projector(board.n_d);
        out.reprojection_error =
            projector.reprojection_error(out.fit_poses, detections, lattice, board.radius, params,
                                         mode);
    }

    // --- held-out frames ----------------------------------------------------
    // Only the detections are recorded here; the poses are re-solved by
    // evaluate() against whatever intrinsics end up final (after a joint
    // refinement the mono poses are no longer the right ones).
    out.holdout_frames.reserve(holdout_positions.size());
    for (std::size_t i = 0; i < holdout_positions.size(); ++i) {
        const BoardFrame& source = accepted_[holdout_positions[i]];
        HeldOutFrame frame;
        frame.stamp_ns = source.stamp_ns;
        frame.own_shapes = source.shapes;
        const std::map<std::int64_t, BoardFrame>::const_iterator other =
            other_.find(source.stamp_ns);
        if (other != other_.end()) frame.other_shapes = other->second.shapes;
        out.holdout_frames.push_back(frame);
    }
    if (out.holdout_frames.empty() && out.holdout_reason.empty()) {
        out.holdout_reason = "no frame was held out to score";
    }

    return out;
}

SolveOutcome CalibrateSession::solve_external(const std::vector<int>& fit_positions,
                                              const std::vector<int>& holdout_positions,
                                              int attempt) {
    // The external solver keeps no intermediate files, so there is no attempt
    // subdirectory to name. The parameter stays for signature parity with the
    // image path and for a future diagnostic dump.
    (void)attempt;

    SolveOutcome out;
    out.pose_source = "external";
    out.n_frames_heldout = static_cast<int>(holdout_positions.size());

    const BoardConfig& board = rig_.board;
    const std::size_t expected =
        static_cast<std::size_t>(board.n_x) * static_cast<std::size_t>(board.n_y);

    // Sorted by stamp so the matcher's temporal-continuity preference walks the
    // collection in time, not in arrival order. A queued feed can deliver a
    // frame late, and a flip decision made against a future frame is noise.
    std::vector<int> ordered = fit_positions;
    std::sort(ordered.begin(), ordered.end(), [this](int a, int b) {
        return accepted_[a].stamp_ns < accepted_[b].stamp_ns;
    });

    std::vector<KnownPoseFrame> inputs;
    inputs.reserve(ordered.size());
    int skipped = 0;
    for (std::size_t i = 0; i < ordered.size(); ++i) {
        const BoardFrame& source = accepted_[ordered[i]];
        if (!source.external.present || source.external.points.size() != expected ||
            source.shapes.size() != expected) {
            ++skipped;
            continue;
        }
        KnownPoseFrame input;
        input.stamp_ns = source.stamp_ns;
        input.shapes = source.shapes;
        input.points = source.external.points;
        inputs.push_back(input);
    }
    out.rejected_external_frames = skipped;

    if (static_cast<int>(inputs.size()) < min_fit_frames()) {
        std::ostringstream oss;
        oss << "only " << inputs.size()
            << " fit frame(s) carried a usable external pose; a " << board.type
            << " board with n_d=" << board.n_d << " cannot be solved from fewer than "
            << min_fit_frames();
        out.message = oss.str();
        return out;
    }

    KnownPoseOptions options;
    options.image_width = width_;
    options.image_height = height_;
    options.projection_mode = calibration_mode(board.type);
    options.min_frames = min_fit_frames();
    options.ceres_threads = rig_.performance.ceres_threads;
    options.external = rig_.external;

    ::Params initial;
    const ::Params* initial_ptr = NULL;
    const CameraConfig* config = rig_.camera(camera_);
    if (config != NULL && config->has_intrinsics) {
        initial = config->intrinsics;
        initial_ptr = &initial;
    }

    KnownPoseSolver solver(board, options);
    KnownPoseResult result;
    solver.solve(inputs, initial_ptr, &result);

    out.params = result.params;
    out.fit_seconds = result.fit_seconds;
    out.matched_frames = result.matched_frames;
    out.ambiguous_frames = result.ambiguous_frames;
    out.rejected_external_frames += result.rejected_frames;
    out.mean_rigidity_rms_m = result.mean_rigidity_rms_m;
    out.initialization = result.initialization;
    out.n_frames = result.matched_frames;
    out.reprojection_error = result.reprojection_error_px;
    out.message = result.message;

    if (!result.ok) {
        return out;
    }
    out.ok = true;

    // The fit set in the solver's (chronological) order.
    out.fit_stamps.reserve(static_cast<std::size_t>(result.matched_frames));
    out.fit_shapes.reserve(static_cast<std::size_t>(result.matched_frames));
    out.fit_poses.reserve(static_cast<std::size_t>(result.matched_frames));
    for (std::size_t i = 0; i < inputs.size() && i < result.frames.size(); ++i) {
        if (!result.frames[i].used) continue;
        out.fit_stamps.push_back(inputs[i].stamp_ns);
        out.fit_shapes.push_back(inputs[i].shapes);
        out.fit_poses.push_back(result.frames[i].match.T);
    }

    // --- held-out frames ----------------------------------------------------
    out.holdout_frames.reserve(holdout_positions.size());
    for (std::size_t i = 0; i < holdout_positions.size(); ++i) {
        const BoardFrame& source = accepted_[holdout_positions[i]];
        HeldOutFrame frame;
        frame.stamp_ns = source.stamp_ns;
        frame.own_shapes = source.shapes;
        frame.has_external = source.external.present;
        frame.own_external_points = source.external.points;

        const std::map<std::int64_t, BoardFrame>::const_iterator other =
            other_.find(source.stamp_ns);
        if (other != other_.end()) {
            frame.other_shapes = other->second.shapes;
            frame.other_has_external = other->second.external.present;
            frame.other_external_points = other->second.external.points;
        }
        out.holdout_frames.push_back(frame);
    }
    if (out.holdout_frames.empty() && out.holdout_reason.empty()) {
        out.holdout_reason = "no frame was held out to score";
    }

    return out;
}

CheckResult CalibrateSession::evaluate_external(const SolveOutcome& outcome,
                                                const ::Params& final_params) const {
    CheckResult check;
    if (!outcome.ok) {
        check.reason = outcome.message.empty() ? "the solve produced no usable intrinsics"
                                               : outcome.message;
        return check;
    }
    if (outcome.holdout_frames.empty()) {
        check.reason = outcome.holdout_reason.empty() ? "no frame was held out to score"
                                                      : outcome.holdout_reason;
        return check;
    }
    if (!check_configured_) {
        check.reason = check_unavailable_reason_;
        return check;
    }
    if (!have_other_params_) {
        check.reason = "the intrinsics of \"" + other_camera_ + "\" are not known";
        return check;
    }

    const BoardConfig& board = rig_.board;
    const int mode = calibration_mode(board.type);
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    if (lattice.empty()) {
        check.reason = "board lattice is empty";
        return check;
    }

    BoardMatchOptions options;
    options.max_rigidity_rms_m = rig_.external.max_rigidity_rms_m;
    options.max_match_rms_px = rig_.external.max_match_rms_px;
    options.min_match_margin_px = rig_.external.min_match_margin_px;
    options.temporal_margin_px = rig_.external.temporal_margin_px;
    options.allow_ambiguous = (rig_.external.ambiguous_policy == "use_best");
    options.projection_mode = mode;

    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);
    const BoardProjector projector(board.n_d);

    // Both cameras are matched independently with their final intrinsics. A
    // match supplies the board pose AND the permutation; reordering both
    // cameras' detections into nominal lattice order is what makes the existing
    // cross-camera check applicable unchanged.
    std::vector<HeldOutFrame> frames;
    frames.reserve(outcome.holdout_frames.size());
    int unusable = 0;

    // Match in time order so the temporal-continuity preference behaves as it
    // does in the solve. The holdout list is in collection order, which a
    // queued feed does not guarantee to be chronological.
    std::vector<const HeldOutFrame*> ordered;
    ordered.reserve(outcome.holdout_frames.size());
    for (std::size_t i = 0; i < outcome.holdout_frames.size(); ++i) {
        ordered.push_back(&outcome.holdout_frames[i]);
    }
    std::sort(ordered.begin(), ordered.end(),
              [](const HeldOutFrame* a, const HeldOutFrame* b) {
                  return a->stamp_ns < b->stamp_ns;
              });

    std::vector<int> previous_own;
    std::vector<int> previous_other;

    for (std::size_t i = 0; i < ordered.size(); ++i) {
        const HeldOutFrame& source = *ordered[i];
        if (!source.has_external || !source.other_has_external) {
            ++unusable;
            continue;
        }
        if (source.own_shapes.size() != lattice.size() ||
            source.other_shapes.size() != lattice.size() ||
            source.own_external_points.size() != lattice.size() ||
            source.other_external_points.size() != lattice.size()) {
            ++unusable;
            continue;
        }

        const BoardMatch own = match_board_pose(
            lattice, source.own_external_points, source.own_shapes, final_params, board.radius,
            projector, options, symmetries,
            previous_own.empty() ? NULL : &previous_own);
        if (!own.ok) {
            ++unusable;
            continue;
        }

        const BoardMatch other = match_board_pose(
            lattice, source.other_external_points, source.other_shapes, other_params_,
            board.radius, projector, options, symmetries,
            previous_other.empty() ? NULL : &previous_other);
        if (!other.ok) {
            ++unusable;
            continue;
        }
        previous_own = own.permutation;
        previous_other = other.permutation;

        HeldOutFrame frame;
        frame.stamp_ns = source.stamp_ns;
        frame.T = own.T;

        // The own detections are already in the order the check wants: the
        // matcher chose own.permutation precisely so that lattice[k] projects
        // to source.own_shapes[k] through own.T. The other camera may have
        // labeled the same board through a different symmetry, so align its
        // observations by original label: the physical dot at own detector k
        // is own.permutation[k], and the other detector index carrying that
        // same label is its inverse image under other.permutation.
        std::vector<int> other_index_by_label(source.other_shapes.size(), -1);
        bool labels_align = true;
        for (std::size_t j = 0; j < other.permutation.size(); ++j) {
            const int label = other.permutation[j];
            if (label < 0 || static_cast<std::size_t>(label) >= other_index_by_label.size()) {
                labels_align = false;
                break;
            }
            other_index_by_label[static_cast<std::size_t>(label)] = static_cast<int>(j);
        }
        if (!labels_align) {
            ++unusable;
            continue;
        }

        // Shape has no default constructor, so build the reordered vector by
        // appending rather than resizing. A failed frame is discarded whole.
        frame.own_shapes = source.own_shapes;
        frame.other_shapes.clear();
        frame.other_shapes.reserve(lattice.size());
        for (std::size_t k = 0; k < lattice.size(); ++k) {
            const int label = own.permutation[k];
            if (label < 0 || static_cast<std::size_t>(label) >= other_index_by_label.size() ||
                other_index_by_label[static_cast<std::size_t>(label)] < 0) {
                labels_align = false;
                break;
            }
            frame.other_shapes.push_back(source.other_shapes[static_cast<std::size_t>(
                other_index_by_label[static_cast<std::size_t>(label)])]);
        }
        if (!labels_align) {
            ++unusable;
            continue;
        }
        frames.push_back(frame);
    }

    if (frames.empty()) {
        std::ostringstream oss;
        oss << "no held-out frame could be matched with its external pose (" << unusable
            << " unusable of " << outcome.holdout_frames.size() << ")";
        check.reason = oss.str();
        return check;
    }

    check = run_cross_camera_check(frames, lattice, board.radius, board.n_d, mode, final_params,
                                   other_params_, T_own_to_other_, rig_.check.max_rms_px);
    if (!check.ran && unusable > 0 && !check.reason.empty()) {
        std::ostringstream oss;
        oss << check.reason << "; " << unusable
            << " held-out frame(s) were also dropped before the check for an unusable external "
               "match";
        check.reason = oss.str();
    }
    return check;
}

CheckResult CalibrateSession::evaluate(const SolveOutcome& outcome,
                                       const ::Params& final_params) const {
    if (rig_.solve.pose_source == "external") {
        return evaluate_external(outcome, final_params);
    }

    CheckResult check;
    if (!outcome.ok) {
        check.reason = outcome.message.empty() ? "the solve produced no usable intrinsics"
                                               : outcome.message;
        return check;
    }
    if (outcome.holdout_frames.empty()) {
        check.reason = outcome.holdout_reason.empty() ? "no frame was held out to score"
                                                      : outcome.holdout_reason;
        return check;
    }
    if (!check_configured_) {
        check.reason = check_unavailable_reason_;
        return check;
    }
    if (!have_other_params_) {
        check.reason = "the intrinsics of \"" + other_camera_ + "\" are not known";
        return check;
    }

    const BoardConfig& board = rig_.board;
    const int mode = calibration_mode(board.type);
    const std::string raw_directory =
        outcome.raw_output_directory.empty() ? output_directory_ : outcome.raw_output_directory;

    // Poses for exactly the held-out frames, with the final intrinsics frozen:
    // Calibrator::update_Es is the same primitive upstream uses for its
    // `fix_intrinsic` mode.
    Calibrator holder(board.n_x, board.n_y, board.asymmetric, board.n_d, board.radius,
                      board.distance, static_cast<int>(outcome.holdout_frames.size()),
                      raw_directory + "/");
    holder.set_image_size(width_, height_);
    for (std::size_t i = 0; i < outcome.holdout_frames.size(); ++i) {
        holder.inputTarget(outcome.holdout_frames[i].own_shapes);
    }
    holder.update_Es(final_params, mode);
    const std::vector<se3> poses = holder.get_extrinsic();

    const std::size_t n_frames = std::min(outcome.holdout_frames.size(), poses.size());
    std::vector<HeldOutFrame> frames(outcome.holdout_frames.begin(),
                                     outcome.holdout_frames.begin() + n_frames);
    for (std::size_t i = 0; i < n_frames; ++i) {
        frames[i].T = LieAlgebra::to_SE3(poses[i]);
    }

    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    return run_cross_camera_check(frames, lattice, board.radius, board.n_d, mode, final_params,
                                  other_params_, T_own_to_other_, rig_.check.max_rms_px);
}

std::string CalibrateSession::camera_info_yaml(const ::Params& params) const {
    const CameraConfig* config = rig_.camera(camera_);
    const std::string frame_id = (config != NULL) ? config->frame_id : std::string();
    return camera_info_to_yaml(
        make_mono_camera_info(params, width_, height_, camera_, frame_id));
}

bool CalibrateSession::write_camera_info(const ::Params& params, const std::string& directory,
                                         std::string* path, std::string* error) const {
    if (!ensure_directory(directory, error)) return false;
    const std::string file = directory + "/" + camera_ + "_camera_info.yaml";
    if (!write_text_file_atomic(file, camera_info_yaml(params), error)) return false;
    if (path != NULL) *path = file;
    return true;
}

SessionResult CalibrateSession::finalize(bool allow_unchecked) {
    SessionResult out;
    ++attempts_;
    out.attempts = attempts_;

    const SolveOutcome solved = solve(NULL, attempts_);
    out.params = solved.params;
    out.n_frames = solved.n_frames;
    out.n_frames_heldout = solved.n_frames_heldout;
    out.reprojection_error = solved.reprojection_error;
    out.fit_seconds = solved.fit_seconds;
    out.raw_output_directory = solved.raw_output_directory;
    out.pose_source = solved.pose_source;
    out.frames_matched = solved.matched_frames;
    out.frames_ambiguous = solved.ambiguous_frames;
    out.frames_rejected_external = solved.rejected_external_frames;
    out.mean_rigidity_rms_m = solved.mean_rigidity_rms_m;

    if (!solved.ok) {
        out.message = solved.message;
        return out;
    }

    out.check = evaluate(solved, solved.params);

    // --- verdict ------------------------------------------------------------
    std::string check_note;
    bool check_ok = false;
    if (out.check.ran) {
        check_ok = out.check.passed;
        check_note = out.check.passed ? "check passed" : out.check.reason;
    } else if (allow_unchecked) {
        check_ok = true;
        check_note = "check did not run (" + out.check.reason + "), allowed by the goal";
    } else {
        check_note = "check did not run: " + out.check.reason +
                     "; pass allow_unchecked to accept a calibration without it";
    }

    std::ostringstream summary;
    summary << "calibrated " << camera_ << " from " << out.n_frames << " frame(s)";
    if (out.n_frames_heldout > 0) summary << " with " << out.n_frames_heldout << " held out";
    summary << "; reprojection error " << format_double(out.reprojection_error) << " px";

    if (!check_ok) {
        out.success = false;
        out.message = summary.str() + "; " + check_note;
        return out;
    }

    // --- write --------------------------------------------------------------
    std::string error;
    std::string path;
    if (!write_camera_info(solved.params, rig_.output.directory, &path, &error)) {
        out.message = summary.str() + "; calibration is good but writing it failed: " + error;
        return out;
    }

    out.camera_info_path = path;
    out.success = true;
    out.message = summary.str() + "; " + check_note + "; wrote " + path;
    return out;
}

}  // namespace discocal_ros
