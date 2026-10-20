#include "discocal_ros/rig_session.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <sys/stat.h>
#include <sys/types.h>

#include "discocal_ros/camera_info_writer.h"
#include "discocal_ros/dwecal_bridge.h"
#include "discocal_ros/dwecal_gen.h"
#include "discocal_ros/sha256.h"

// Vendored headers. CStereoCalibrator.h and CLieAlgebra.h do `using namespace
// std;` at global scope, so they stay confined to this translation unit.
#include "CStereoCalibrator.h"
#include "CLieAlgebra.h"

namespace discocal_ros {
namespace {

std::string format_double(double value, int precision = 6) {
    std::ostringstream oss;
    oss << std::setprecision(precision) << value;
    return oss.str();
}

// 17 significant digits round-trips a double. Every number written into a run
// artifact uses this, so a later reader gets the solver's value, not a
// six-digit shadow of it.
std::string exact(double value) {
    std::ostringstream oss;
    oss << std::setprecision(17) << value;
    return oss.str();
}

std::string quote(const std::string& value) {
    std::ostringstream oss;
    oss << '"';
    for (std::size_t i = 0; i < value.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(value[i]);
        switch (c) {
            case '\\':
                oss << "\\\\";
                break;
            case '"':
                oss << "\\\"";
                break;
            case '\n':
                oss << "\\n";
                break;
            case '\r':
                oss << "\\r";
                break;
            case '\t':
                oss << "\\t";
                break;
            default:
                if (c < 0x20) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof(buffer), "\\x%02x", c);
                    oss << buffer;
                } else {
                    oss << static_cast<char>(c);
                }
                break;
        }
    }
    oss << '"';
    return oss.str();
}

std::string boolean(bool value) { return value ? "true" : "false"; }

bool local_time(std::time_t when, std::tm* out) {
    return localtime_r(&when, out) != NULL;
}

std::string iso8601(std::time_t when) {
    std::tm tm;
    if (!local_time(when, &tm)) return "1970-01-01T00:00:00";
    char buffer[32];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &tm) == 0) {
        return "1970-01-01T00:00:00";
    }
    return buffer;
}

bool parameters_usable(const ::Params& params) {
    for (int i = 0; i < 5; ++i) {
        if (!std::isfinite(params.K[i])) return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(params.d[i])) return false;
    }
    return params.fx() > 0.0 && params.fy() > 0.0;
}

bool same_vector3(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    // Bit-identical on purpose: a fixed extrinsic must come back untouched.
    for (int i = 0; i < 3; ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

std::string land_intrinsics_text(const ::Params& params, const BoardConfig& board) {
    std::ostringstream oss;
    oss << "n_d: " << board.n_d << "\n";
    oss << "radius: " << exact(board.radius) << "\n";
    oss << "distance: " << exact(board.distance) << "\n";
    oss << "fx: " << exact(params.K[0]) << "\n";
    oss << "fy: " << exact(params.K[1]) << "\n";
    oss << "cx: " << exact(params.K[2]) << "\n";
    oss << "cy: " << exact(params.K[3]) << "\n";
    oss << "skew: " << exact(params.K[4]) << "\n";
    oss << "d1: " << exact(params.d[0]) << "\n";
    oss << "d2: " << exact(params.d[1]) << "\n";
    oss << "d3: " << exact(params.d[2]) << "\n";
    oss << "d4: " << exact(params.d[3]) << "\n";
    oss << "sfx: " << exact(params.s_K[0]) << "\n";
    oss << "sfy: " << exact(params.s_K[1]) << "\n";
    oss << "scx: " << exact(params.s_K[2]) << "\n";
    oss << "scy: " << exact(params.s_K[3]) << "\n";
    oss << "sskew: " << exact(params.s_K[4]) << "\n";
    oss << "sd1: " << exact(params.s_d[0]) << "\n";
    oss << "sd2: " << exact(params.s_d[1]) << "\n";
    oss << "sd3: " << exact(params.s_d[2]) << "\n";
    oss << "sd4: " << exact(params.s_d[3]) << "\n";
    return oss.str();
}

std::string land_poses_text(const ExtrinsicConfig& extrinsic) {
    // gen_land.cc's format: R_land = R_config^T, T_land = -R_land * t_config,
    // with the rotation printed as a Rodrigues vector in DEGREES. This is the
    // exact inverse of the conversion the loader performs.
    const Eigen::Matrix3d R_land = extrinsic.R.transpose();
    const Eigen::Vector3d T_land = -R_land * extrinsic.t;
    const Eigen::AngleAxisd angle_axis(R_land);
    const Eigen::Vector3d rvec = angle_axis.axis() * angle_axis.angle();
    const double degrees = 180.0 / M_PI;

    std::ostringstream oss;
    oss << "Relative poses described in camera0 coordinates (p_0 = T_i^0 p_i)\n";
    oss << "r_x\t\tr_y\t\tr_z\t\tt_x\t\tt_y\t\tt_z\n";
    oss << std::fixed << std::setprecision(9);
    oss << "T_1^:\t" << rvec[0] * degrees << "\t" << rvec[1] * degrees << "\t"
        << rvec[2] * degrees << "\t" << T_land[0] << "\t" << T_land[1] << "\t" << T_land[2]
        << "\n";
    return oss.str();
}

void write_check_yaml(std::ostringstream& oss, const CheckResult& check) {
    oss << "      ran: " << boolean(check.ran) << "\n";
    oss << "      passed: " << boolean(check.passed) << "\n";
    oss << "      reason: " << quote(check.reason) << "\n";
    oss << "      pairs: " << check.pairs << "\n";
    oss << "      points: " << check.points << "\n";
    oss << "      rms_px: " << exact(check.rms_px) << "\n";
    oss << "      worst_pair_rms_px: " << exact(check.worst_pair_rms_px) << "\n";
    oss << "      limit_px: " << exact(check.limit_px) << "\n";
    oss << "      pairs_dropped: " << check.pairs_dropped << "\n";
    oss << "      pairs_unpaired: " << check.pairs_unpaired << "\n";
}

}  // namespace

std::vector<std::int64_t> choose_common_holdout(
    const std::vector<std::int64_t>& accepted_a, const std::vector<std::int64_t>& accepted_b,
    double fraction, int min_fit_a, int min_fit_b) {
    std::vector<std::int64_t> a = accepted_a;
    std::vector<std::int64_t> b = accepted_b;
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
    std::sort(b.begin(), b.end());
    b.erase(std::unique(b.begin(), b.end()), b.end());

    std::vector<std::int64_t> paired;
    std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(paired));
    if (paired.empty() || !std::isfinite(fraction) || fraction <= 0.0) {
        return std::vector<std::int64_t>();
    }

    // A camera that accepted fewer frames than its fit floor must not be
    // starved by a shared holdout, even if the other camera is comfortable.
    long long room_a = static_cast<long long>(a.size()) - std::max(0, min_fit_a);
    long long room_b = static_cast<long long>(b.size()) - std::max(0, min_fit_b);
    long long room = std::min(static_cast<long long>(paired.size()),
                              std::min(room_a, room_b));
    if (room <= 0) return std::vector<std::int64_t>();

    const long long requested = std::llround(fraction * static_cast<double>(paired.size()));
    const long long n_holdout = std::max(0ll, std::min(requested, room));
    if (n_holdout == 0) return std::vector<std::int64_t>();

    const std::vector<int> positions =
        spread_indices(static_cast<int>(paired.size()), static_cast<int>(n_holdout));
    std::vector<std::int64_t> out;
    out.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        out.push_back(paired[positions[i]]);
    }
    return out;
}

bool create_timestamped_run_directory(const std::string& base, std::string* directory,
                                      std::string* error) {
    if (directory == NULL) {
        if (error != NULL) *error = "directory pointer is null";
        return false;
    }
    std::string detail;
    if (!ensure_directory(base, &detail)) {
        if (error != NULL) *error = detail;
        return false;
    }

    const std::time_t now = std::time(NULL);
    std::tm tm;
    char stamp[32] = "19700101_000000";
    if (local_time(now, &tm)) {
        if (std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm) == 0) {
            std::strcpy(stamp, "19700101_000000");
        }
    }

    for (int suffix = 0; suffix < 1000; ++suffix) {
        std::ostringstream name;
        name << base;
        if (!base.empty() && base[base.size() - 1] != '/') name << '/';
        name << "calibration_" << stamp;
        if (suffix > 0) name << '_' << suffix;

        const std::string candidate = name.str();
        if (::mkdir(candidate.c_str(), 0777) == 0) {
            *directory = candidate;
            return true;
        }
        if (errno != EEXIST) {
            if (error != NULL) {
                *error = "cannot create " + candidate + ": " + std::strerror(errno);
            }
            return false;
        }
    }
    if (error != NULL) {
        *error = "could not find an unused calibration directory name under " + base;
    }
    return false;
}

RigSession::RigSession(const RigConfig& rig, const std::vector<std::string>& cameras, int budget,
                       const std::map<std::string, ::Params>& known_intrinsics)
    : rig_(rig), cameras_(cameras), budget_(budget > 0 ? budget : rig.frames.budget) {
    if (cameras_.size() != 2) {
        throw std::runtime_error("both-camera calibration requires exactly two cameras, not " +
                                 std::to_string(cameras_.size()));
    }
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        const std::string& name = cameras_[i];
        if (states_.find(name) != states_.end()) {
            throw std::runtime_error("camera \"" + name + "\" is listed twice");
        }
        const CameraConfig* config = rig_.camera(name);
        if (config == NULL) {
            throw std::runtime_error("camera \"" + name + "\" is not in the rig config");
        }
        if (config->source != "calibrate") {
            throw std::runtime_error("camera \"" + name +
                                     "\" has source: " + config->source +
                                     "; both-camera calibration needs source: calibrate");
        }

        CameraState& camera_state = states_[name];
        camera_state.session.reset(new CalibrateSession(rig_, name, budget_, known_intrinsics));
    }
}

RigSession::CameraState& RigSession::state(const std::string& camera) {
    std::map<std::string, CameraState>::iterator it = states_.find(camera);
    if (it == states_.end()) {
        throw std::out_of_range("no such camera in this both-camera session: " + camera);
    }
    return it->second;
}

const RigSession::CameraState& RigSession::state(const std::string& camera) const {
    std::map<std::string, CameraState>::const_iterator it = states_.find(camera);
    if (it == states_.end()) {
        throw std::out_of_range("no such camera in this both-camera session: " + camera);
    }
    return it->second;
}

std::vector<std::string> RigSession::parent_child_order() const {
    if (!rig_.extrinsic.present) return std::vector<std::string>();
    if (rig_.extrinsic.parent == rig_.extrinsic.child) return std::vector<std::string>();
    if (states_.find(rig_.extrinsic.parent) == states_.end() ||
        states_.find(rig_.extrinsic.child) == states_.end()) {
        return std::vector<std::string>();
    }
    std::vector<std::string> order;
    order.push_back(rig_.extrinsic.parent);
    order.push_back(rig_.extrinsic.child);
    return order;
}

void RigSession::set_image_size(const std::string& camera, int width, int height) {
    state(camera).session->set_image_size(width, height);
}

bool RigSession::has_image_size(const std::string& camera) const {
    return state(camera).session->has_image_size();
}

bool RigSession::on_detection(const BoardFrame& frame) {
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& own = state(cameras_[i]);
        if (cameras_[i] == frame.camera) {
            // A camera that filled its own budget stops accepting, but its
            // frames keep feeding the other camera's pairing history.
            if (!own.session->budget_reached()) own.session->on_detection(frame);
        } else {
            own.session->on_other_detection(frame);
        }
    }
    return budget_reached();
}

bool RigSession::budget_reached() const {
    if (states_.empty()) return true;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        if (!state(cameras_[i]).session->budget_reached()) return false;
    }
    return true;
}

const SessionStats& RigSession::stats(const std::string& camera) const {
    return state(camera).session->stats();
}

int RigSession::min_fit_frames(const std::string& camera) const {
    return state(camera).session->min_fit_frames();
}

void RigSession::reset() {
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        state(cameras_[i]).session->reset();
    }
}

RigSessionResult RigSession::solve(bool allow_unchecked, const InterruptCheck& interrupt) {
    RigSessionResult out;
    out.attempts = ++attempts_;
    // Polled between stages only. A solve in flight is never torn down: the
    // caller gets a clear "interrupted" result and the action reports preempted.
    const auto interrupted = [&interrupt]() { return interrupt && interrupt(); };

    // Retry state is per attempt. Without this, an attempt whose mono solve
    // fails would leave the previous attempt's final params (and its passing
    // check) in place, and the artifact pass below would happily write a
    // calibration this goal just rejected.
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& st = state(cameras_[i]);
        st.final_params = ::Params();
        st.has_final_params = false;
        st.uncertainties_from_mono_stage = false;
        st.check = CheckResult();
    }

    std::string error;
    if (run_directory_.empty()) {
        if (!create_timestamped_run_directory(rig_.output.directory, &run_directory_, &error)) {
            out.message = "cannot create the run directory under " + rig_.output.directory + ": " +
                          error;
            return out;
        }
        started_at_ = iso8601(std::time(NULL));
    }
    out.run_directory = run_directory_;
    out.land_directory = run_directory_ + "/land";
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        state(cameras_[i]).session->set_output_directory(run_directory_);
    }
    if (interrupted()) {
        out.message = "interrupted before the mono solves";
        return out;
    }

    // --- the shared holdout ------------------------------------------------
    std::vector<std::int64_t> stamps_a;
    std::vector<std::int64_t> stamps_b;
    const std::vector<BoardFrame>& accepted_a = state(cameras_[0]).session->accepted_frames();
    const std::vector<BoardFrame>& accepted_b = state(cameras_[1]).session->accepted_frames();
    for (std::size_t i = 0; i < accepted_a.size(); ++i) stamps_a.push_back(accepted_a[i].stamp_ns);
    for (std::size_t i = 0; i < accepted_b.size(); ++i) stamps_b.push_back(accepted_b[i].stamp_ns);

    const std::vector<std::int64_t> shared = choose_common_holdout(
        stamps_a, stamps_b, rig_.frames.holdout_fraction,
        state(cameras_[0]).session->min_fit_frames(),
        state(cameras_[1]).session->min_fit_frames());
    if (shared.empty() && rig_.frames.holdout_fraction > 0.0) {
        out.warnings.push_back(
            "no stamp is available for a shared holdout; the cross-camera check cannot run");
    }
    if (!shared.empty()) {
        out.warnings.push_back("shared holdout: " + std::to_string(shared.size()) +
                               " frame(s) accepted by both cameras");
    }

    // --- mono solves --------------------------------------------------------
    bool all_solved = true;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& st = state(cameras_[i]);
        StageClock solve_clock;
        st.solved = st.session->solve(&shared, attempts_);
        out.timings.mono_solve[cameras_[i]] = solve_clock.stop();
        out.timings.mono_fit_seconds += st.solved.fit_seconds;

        CameraOutcome outcome;
        outcome.camera = cameras_[i];
        outcome.result.attempts = attempts_;
        outcome.result.params = st.solved.params;
        outcome.result.n_frames = st.solved.n_frames;
        outcome.result.n_frames_heldout = st.solved.n_frames_heldout;
        outcome.result.reprojection_error = st.solved.reprojection_error;
        outcome.result.fit_seconds = st.solved.fit_seconds;
        outcome.result.raw_output_directory = st.solved.raw_output_directory;
        if (!st.solved.ok) {
            outcome.result.message = st.solved.message;
            all_solved = false;
        } else {
            st.final_params = st.solved.params;
            st.has_final_params = true;
        }
        out.cameras.push_back(outcome);
    }
    if (interrupted()) {
        out.message = "interrupted during the mono solves";
        return out;
    }

    // --- optional joint refinement, extrinsic fixed ------------------------
    if (interrupted()) {
        out.message = "interrupted before the joint refinement";
        return out;
    }

    const std::vector<std::string> order = parent_child_order();
    bool joint_failure_fatal = false;
    std::string joint_failure;
    bool joint_attempted = false;

    if (rig_.solve.joint_refine && all_solved) {
        if (rig_.board.type != "circle") {
            out.warnings.push_back(
                "joint refinement is only implemented for circle boards; using the two mono "
                "results");
        } else if (order.empty()) {
            out.warnings.push_back(
                "joint refinement skipped: the configured extrinsic does not name both cameras");
        } else {
            // From here on, a failure is the joint refinement's failure rather
            // than "not applicable", so joint_refine_fallback decides whether it
            // fails the goal.
            joint_attempted = true;
            StageClock joint_clock;
            CameraState& parent = state(order[0]);
            CameraState& child = state(order[1]);

            std::map<std::int64_t, std::size_t> child_fit;
            for (std::size_t i = 0; i < child.solved.fit_stamps.size(); ++i) {
                child_fit.insert(std::make_pair(child.solved.fit_stamps[i], i));
            }

            std::vector<Target> parent_targets;
            std::vector<Target> child_targets;
            std::vector<se3> scene_poses;
            for (std::size_t i = 0; i < parent.solved.fit_stamps.size(); ++i) {
                const std::map<std::int64_t, std::size_t>::const_iterator it =
                    child_fit.find(parent.solved.fit_stamps[i]);
                if (it == child_fit.end()) continue;
                if (i >= parent.solved.fit_poses.size()) break;
                if (it->second >= child.solved.fit_shapes.size()) continue;
                if (parent.solved.fit_shapes[i].size() !=
                    child.solved.fit_shapes[it->second].size()) {
                    continue;
                }
                parent_targets.push_back(Target(true, parent.solved.fit_shapes[i]));
                child_targets.push_back(Target(true, child.solved.fit_shapes[it->second]));
                scene_poses.push_back(LieAlgebra::to_se3(parent.solved.fit_poses[i]));
            }

            const int min_fit = std::max(parent.session->min_fit_frames(),
                                         child.session->min_fit_frames());
            if (static_cast<int>(parent_targets.size()) < min_fit) {
                joint_failure = "only " + std::to_string(parent_targets.size()) +
                                " scene(s) were fitted by both cameras; " +
                                std::to_string(min_fit) + " are required";
            } else {
                std::vector<std::vector<Target> > all_targets;
                all_targets.push_back(parent_targets);
                all_targets.push_back(child_targets);

                std::vector<::Params> all_params;
                all_params.push_back(parent.solved.params);
                all_params.push_back(child.solved.params);

                std::vector<double> all_radius(2, rig_.board.radius);
                std::vector<double> all_n_d(2, static_cast<double>(rig_.board.n_d));

                // The config convention: p_parent = R * p_child + t. That is
                // exactly the direction SubCalibrationFunctor takes (see the
                // derivation in its header): no inversion here.
                std::vector<se3> all_ext(1);
                all_ext[0].rot = LieAlgebra::to_so3(rig_.extrinsic.R);
                all_ext[0].trans = rig_.extrinsic.t;
                const se3 given_extrinsic = all_ext[0];

                bool converged = false;
                const std::chrono::steady_clock::time_point start =
                    std::chrono::steady_clock::now();
                try {
                    StereoCalibrator calibrator(rig_.board.n_x, rig_.board.n_y,
                                                rig_.board.asymmetric, rig_.board.distance);
                    calibrator.calibrate(all_targets, scene_poses, all_ext, all_params,
                                         all_radius, all_n_d,
                                         true,   // full_optimize
                                         true,   // fix_extrinsic
                                         &converged);
                } catch (const std::exception& e) {
                    joint_failure = std::string("joint refinement threw: ") + e.what();
                }
                out.joint_fit_seconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                        .count();

                if (joint_failure.empty() && !converged) {
                    joint_failure = "Ceres did not report convergence";
                }
                if (joint_failure.empty() &&
                    (!parameters_usable(all_params[0]) || !parameters_usable(all_params[1]))) {
                    joint_failure = "the joint solve returned non-finite or non-positive intrinsics";
                }
                if (joint_failure.empty() &&
                    (!same_vector3(all_ext[0].rot, given_extrinsic.rot) ||
                     !same_vector3(all_ext[0].trans, given_extrinsic.trans))) {
                    // The extrinsic is an input, not a parameter. If it moved,
                    // the result cannot be trusted under this contract.
                    joint_failure =
                        "the joint solve modified the fixed extrinsic; refusing its result";
                }

                if (joint_failure.empty()) {
                    parent.final_params = all_params[0];
                    child.final_params = all_params[1];
                    parent.has_final_params = true;
                    child.has_final_params = true;
                    parent.uncertainties_from_mono_stage = true;
                    child.uncertainties_from_mono_stage = true;
                    out.joint_refined = true;
                }
            }
            out.timings.joint_refine = joint_clock.stop();
        }
    }
    out.timings.joint_fit_seconds = out.joint_fit_seconds;

    if (interrupted()) {
        out.message = "interrupted after the joint refinement";
        return out;
    }

    if (joint_attempted && !out.joint_refined) {
        if (rig_.solve.joint_refine_fallback) {
            out.warnings.push_back("joint refinement failed (" + joint_failure +
                                   "); using the two mono results");
        } else {
            joint_failure_fatal = true;
            out.warnings.push_back("joint refinement failed (" + joint_failure +
                                   "); solve.joint_refine_fallback is false, so the goal fails");
        }
    }

    // Publish the final params and the mono-stage uncertainty flag.
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& st = state(cameras_[i]);
        out.cameras[i].uncertainties_from_mono_stage = st.uncertainties_from_mono_stage;
        if (st.has_final_params) out.cameras[i].result.params = st.final_params;
    }

    if (interrupted()) {
        out.message = "interrupted before evaluation";
        return out;
    }

    // --- evaluate each camera against the other's FINAL params -------------
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        for (std::size_t j = 0; j < cameras_.size(); ++j) {
            if (i == j) continue;
            if (state(cameras_[j]).has_final_params) {
                state(cameras_[i]).session->set_other_params(
                    state(cameras_[j]).final_params);
            }
        }
    }

    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& st = state(cameras_[i]);
        CameraOutcome& outcome = out.cameras[i];
        if (!st.solved.ok) continue;

        if (!st.has_final_params || !parameters_usable(st.final_params)) {
            outcome.result.success = false;
            outcome.result.message =
                st.solved.message.empty() ? "no usable final intrinsics"
                                          : st.solved.message;
            continue;
        }

        StageClock evaluate_clock;
        st.check = st.session->evaluate(st.solved, st.final_params);
        out.timings.evaluate[cameras_[i]] = evaluate_clock.stop();
        outcome.result.check = st.check;

        bool check_ok = false;
        std::string check_note;
        if (st.check.ran) {
            check_ok = st.check.passed;
            check_note = st.check.passed
                             ? "cross-camera check passed (" +
                                   format_double(st.check.rms_px, 4) + " px, limit " +
                                   format_double(st.check.limit_px, 4) + " px)"
                             : st.check.reason;
        } else if (allow_unchecked) {
            check_ok = true;
            check_note = "check did not run (" + st.check.reason + "), allowed by the goal";
        } else {
            check_note = "check did not run: " + st.check.reason +
                         "; pass allow_unchecked to accept a calibration without it";
        }

        std::ostringstream summary;
        summary << "calibrated " << cameras_[i] << " from " << st.solved.n_frames << " frame(s)";
        if (st.solved.n_frames_heldout > 0) {
            summary << " with " << st.solved.n_frames_heldout << " held out";
        }
        summary << "; reprojection error " << format_double(st.solved.reprojection_error, 4)
                << " px; " << check_note;
        outcome.result.success = check_ok;
        outcome.result.message = summary.str();
    }

    if (interrupted()) {
        out.message = "interrupted before writing artifacts";
        return out;
    }

    // --- artifacts ----------------------------------------------------------
    StageClock artifacts_clock;
    bool artifacts_ok = true;
    std::string artifact_error;

    if (ensure_directory(out.land_directory, &error)) {
        // The land directory exists even if the writes below are skipped; it is
        // part of the run's layout and makes "nothing was written" obvious.
    } else {
        artifacts_ok = false;
        artifact_error = "cannot create " + out.land_directory + ": " + error;
    }

    // Both camera_info files are staged to temp names first and only renamed
    // once every write succeeded, so a full disk cannot leave a run that looks
    // like it produced one camera's calibration but not the other's.
    std::vector<TextFileWrite> camera_info_files;
    std::vector<std::size_t> camera_info_indices;
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraState& st = state(cameras_[i]);
        if (!st.solved.ok || !st.has_final_params || !parameters_usable(st.final_params)) {
            continue;
        }
        TextFileWrite file;
        file.path = run_directory_ + "/" + cameras_[i] + "_camera_info.yaml";
        file.text = st.session->camera_info_yaml(st.final_params);
        camera_info_files.push_back(file);
        camera_info_indices.push_back(i);
    }
    if (!camera_info_files.empty()) {
        std::string write_error;
        if (!write_text_files_atomic(camera_info_files, &write_error)) {
            artifacts_ok = false;
            artifact_error = write_error;
            for (std::size_t k = 0; k < camera_info_indices.size(); ++k) {
                CameraOutcome& camera = out.cameras[camera_info_indices[k]];
                camera.result.success = false;
                camera.result.message += "; writing camera_info failed: " + write_error;
            }
        } else {
            for (std::size_t k = 0; k < camera_info_indices.size(); ++k) {
                CameraOutcome& camera = out.cameras[camera_info_indices[k]];
                camera.result.camera_info_path = camera_info_files[k].path;
                camera.result.message += "; wrote " + camera_info_files[k].path;
            }
        }
    }

    bool both_final = !order.empty();
    if (both_final) {
        for (std::size_t i = 0; i < order.size(); ++i) {
            const CameraState& st = state(order[i]);
            if (!st.has_final_params || !parameters_usable(st.final_params)) both_final = false;
        }
    }

    if (both_final) {
        std::vector<TextFileWrite> land_files(3);
        land_files[0].path = out.land_directory + "/camera0_intrinsic.yaml";
        land_files[0].text =
            land_intrinsics_text(state(order[0]).final_params, rig_.board);
        land_files[1].path = out.land_directory + "/camera1_intrinsic.yaml";
        land_files[1].text =
            land_intrinsics_text(state(order[1]).final_params, rig_.board);
        land_files[2].path = out.land_directory + "/relative_camera_poses.txt";
        land_files[2].text = land_poses_text(rig_.extrinsic);
        std::string land_error;
        if (!write_text_files_atomic(land_files, &land_error)) {
            artifacts_ok = false;
            artifact_error = land_error;
        }
    } else if (rig_.dwecal.enabled) {
        out.warnings.push_back(
            "land-format output and .dwecal skipped: both cameras did not produce finite "
            "intrinsics");
    }

    // .dwecal, its self-checks and the hash sidecar.
    bool dwecal_written = false;
    bool dwecal_generated = false;
    DwecalResult dwecal_result;
    std::string dwecal_reason;
    StageClock dwecal_clock;
    if (rig_.dwecal.enabled) {
        if (!both_final) {
            dwecal_reason = "both cameras did not produce finite intrinsics";
        } else {
            const std::size_t w0 = static_cast<std::size_t>(
                state(order[0]).session->image_width());
            const std::size_t h0 = static_cast<std::size_t>(
                state(order[0]).session->image_height());
            const std::size_t w1 = static_cast<std::size_t>(
                state(order[1]).session->image_width());
            const std::size_t h1 = static_cast<std::size_t>(
                state(order[1]).session->image_height());
            if (w0 == 0 || h0 == 0 || w1 == 0 || h1 == 0) {
                dwecal_reason = "the source image size was never learned";
            } else if (w0 != w1 || h0 != h1) {
                dwecal_reason = "the two cameras calibrated at different source image sizes (" +
                                std::to_string(w0) + "x" + std::to_string(h0) + " and " +
                                std::to_string(w1) + "x" + std::to_string(h1) + ")";
            } else {
                const DwecalInput input = dwecal_input_from_config(
                    state(order[0]).final_params, state(order[1]).final_params, rig_.extrinsic);
                const DwecalStatus status =
                    generate_dwecal(input, static_cast<int>(w0), static_cast<int>(h0),
                                    rig_.dwecal.output_width, rig_.dwecal.output_height,
                                    &dwecal_result, &dwecal_reason);
                if (status != DwecalStatus::kOk) {
                    dwecal_reason = std::string("dwecal generation failed: ") +
                                    dwecal_status_message(status) +
                                    (dwecal_reason.empty() ? "" : ": " + dwecal_reason);
                } else {
                    dwecal_generated = true;
                    const std::uint64_t expected_bytes =
                        30ull + 2ull * static_cast<std::uint64_t>(rig_.dwecal.output_width) *
                                    static_cast<std::uint64_t>(rig_.dwecal.output_height) * 8ull;
                    if (dwecal_result.bytes.size() != expected_bytes) {
                        dwecal_reason = "self-check failed: generated " +
                                        std::to_string(dwecal_result.bytes.size()) +
                                        " bytes, expected " + std::to_string(expected_bytes);
                    } else if (dwecal_result.resolution_byte !=
                               dwecal_resolution_byte(rig_.dwecal.output_width,
                                                      rig_.dwecal.output_height)) {
                        dwecal_reason = "self-check failed: resolution byte does not match the "
                                        "configured output size";
                    } else if (!(dwecal_result.fx > 0.0f) || !(dwecal_result.fy > 0.0f)) {
                        dwecal_reason =
                            "self-check failed: rectified P0 has a non-positive focal length";
                    } else {
                        const std::string path =
                            run_directory_ + "/" + rig_.dwecal.basename;
                        std::string write_error;
                        if (!write_bytes_atomic(path, dwecal_result.bytes, &write_error)) {
                            artifacts_ok = false;
                            artifact_error = write_error;
                            dwecal_reason = write_error;
                        } else {
                            const std::string hash = sha256_hex(
                                dwecal_result.bytes.data(), dwecal_result.bytes.size());
                            const std::string sidecar = path + ".sha256";
                            const std::string sidecar_text =
                                hash + "  " + rig_.dwecal.basename + "\n";
                            if (!write_text_file_atomic(sidecar, sidecar_text, &write_error)) {
                                artifacts_ok = false;
                                artifact_error = write_error;
                                dwecal_reason = write_error;
                            } else {
                                dwecal_written = true;
                                out.dwecal_path = path;
                                out.dwecal_sha256 = hash;
                            }
                        }
                    }
                }
            }
        }
        if (!dwecal_written) {
            out.warnings.push_back("dwecal not written: " + dwecal_reason);
        } else if (dwecal_result.in_bounds_fraction < 0.95) {
            std::ostringstream oss;
            oss << "dwecal map in-bounds fraction is "
                << format_double(dwecal_result.in_bounds_fraction, 4)
                << "; a value well below 1.0 suggests a bad calibration or a wrong extrinsic";
            out.warnings.push_back(oss.str());
        }
    }
    out.timings.dwecal = dwecal_clock.stop();

    // --- overall verdict and run summary -----------------------------------
    bool all_cameras_ok = !out.cameras.empty();
    for (std::size_t i = 0; i < out.cameras.size(); ++i) {
        if (!out.cameras[i].result.success) all_cameras_ok = false;
    }
    out.success = all_cameras_ok && artifacts_ok && !joint_failure_fatal;

    std::ostringstream message;
    if (out.success) {
        message << "calibrated " << cameras_[0] << " and " << cameras_[1];
        message << "; shared holdout " << shared.size() << " frame(s)";
        message << "; joint refinement " << (out.joint_refined ? "applied" : "not applied");
        message << "; wrote " << run_directory_;
    } else {
        message << "both-camera calibration failed";
        if (!all_cameras_ok) {
            message << ": ";
            bool first = true;
            for (std::size_t i = 0; i < out.cameras.size(); ++i) {
                if (out.cameras[i].result.success) continue;
                if (!first) message << "; ";
                first = false;
                message << out.cameras[i].camera << ": " << out.cameras[i].result.message;
            }
            if (first) message << "no camera succeeded";
        }
        if (!artifacts_ok) {
            message << "; artifact error: " << artifact_error;
        }
        if (joint_failure_fatal) {
            message << "; joint refinement failed and solve.joint_refine_fallback is false: "
                    << joint_failure;
        }
    }
    out.message = message.str();

    // The per-camera messages already carry the artifact path; the overall
    // message carries the run directory. rig_calibration.yaml is written last
    // so it can include the .dwecal hash.
    // The top-level verdict describes the RIG, so it requires every camera's
    // check to have run and passed. "One camera checked out" is not a checked
    // rig: the other camera's intrinsics are exactly as unverified as before.
    bool check_ran = !out.cameras.empty();
    bool check_passed = true;
    for (std::size_t i = 0; i < out.cameras.size(); ++i) {
        if (!out.cameras[i].result.check.ran) {
            check_ran = false;
        } else if (!out.cameras[i].result.check.passed) {
            check_passed = false;
        }
    }
    if (!check_ran) check_passed = false;

    std::ostringstream yaml;
    yaml << "run:\n";
    yaml << "  started_at: " << quote(started_at_) << "\n";
    yaml << "  finished_at: " << quote(iso8601(std::time(NULL))) << "\n";
    yaml << "  attempts: " << attempts_ << "\n";
    yaml << "  success: " << boolean(out.success) << "\n";
    yaml << "  check_ran: " << boolean(check_ran) << "\n";
    yaml << "  check_passed: " << boolean(check_passed) << "\n";
    yaml << "  message: " << quote(out.message) << "\n";
    yaml << "  joint_refined: " << boolean(out.joint_refined) << "\n";
    yaml << "  joint_fit_seconds: " << exact(out.joint_fit_seconds) << "\n";
    yaml << "  shared_holdout_frames: " << shared.size() << "\n";
    yaml << "  run_directory: " << quote(run_directory_) << "\n";
    if (out.warnings.empty()) {
        yaml << "  warnings: []\n";
    } else {
        yaml << "  warnings:\n";
        for (std::size_t i = 0; i < out.warnings.size(); ++i) {
            yaml << "    - " << quote(out.warnings[i]) << "\n";
        }
    }

    yaml << "extrinsic:\n";
    yaml << "  present: " << boolean(rig_.extrinsic.present) << "\n";
    yaml << "  parent: " << quote(rig_.extrinsic.parent) << "\n";
    yaml << "  child: " << quote(rig_.extrinsic.child) << "\n";
    yaml << "  rotation: [" << exact(rig_.extrinsic.R(0, 0)) << ", "
         << exact(rig_.extrinsic.R(0, 1)) << ", " << exact(rig_.extrinsic.R(0, 2)) << ", "
         << exact(rig_.extrinsic.R(1, 0)) << ", " << exact(rig_.extrinsic.R(1, 1)) << ", "
         << exact(rig_.extrinsic.R(1, 2)) << ", " << exact(rig_.extrinsic.R(2, 0)) << ", "
         << exact(rig_.extrinsic.R(2, 1)) << ", " << exact(rig_.extrinsic.R(2, 2)) << "]\n";
    yaml << "  translation: [" << exact(rig_.extrinsic.t[0]) << ", "
         << exact(rig_.extrinsic.t[1]) << ", " << exact(rig_.extrinsic.t[2]) << "]\n";
    yaml << "  parent_is_camera0: " << boolean(!order.empty()) << "\n";
    yaml << "  camera1_position_in_camera0: [" << exact(rig_.extrinsic.t[0]) << ", "
         << exact(rig_.extrinsic.t[1]) << ", " << exact(rig_.extrinsic.t[2]) << "]\n";
    yaml << "  baseline_m: " << exact(rig_.extrinsic.t.norm()) << "\n";
    yaml << "  camera1_is_right: " << boolean(rig_.extrinsic.t[0] > 0.0) << "\n";

    yaml << "dwecal:\n";
    yaml << "  enabled: " << boolean(rig_.dwecal.enabled) << "\n";
    yaml << "  written: " << boolean(dwecal_written) << "\n";
    yaml << "  path: " << quote(out.dwecal_path) << "\n";
    yaml << "  sha256: " << quote(out.dwecal_sha256) << "\n";
    yaml << "  width: " << rig_.dwecal.output_width << "\n";
    yaml << "  height: " << rig_.dwecal.output_height << "\n";
    if (dwecal_generated) {
        yaml << "  resolution_byte: " << dwecal_result.resolution_byte << "\n";
        yaml << "  fx: " << exact(dwecal_result.fx) << "\n";
        yaml << "  fy: " << exact(dwecal_result.fy) << "\n";
        yaml << "  cx: " << exact(dwecal_result.cx) << "\n";
        yaml << "  cy: " << exact(dwecal_result.cy) << "\n";
        yaml << "  baseline_m: " << exact(dwecal_result.baseline_m) << "\n";
        yaml << "  camera1_is_right: " << boolean(dwecal_result.camera1_is_right) << "\n";
        yaml << "  in_bounds_fraction: " << exact(dwecal_result.in_bounds_fraction) << "\n";
    }
    if (!dwecal_written) yaml << "  reason: " << quote(dwecal_reason) << "\n";

    yaml << "cameras:\n";
    for (std::size_t i = 0; i < out.cameras.size(); ++i) {
        const CameraOutcome& camera = out.cameras[i];
        const ::Params& params = camera.result.params;
        yaml << "  " << quote(camera.camera) << ":\n";
        yaml << "    success: " << boolean(camera.result.success) << "\n";
        yaml << "    message: " << quote(camera.result.message) << "\n";
        yaml << "    params:\n";
        yaml << "      fx: " << exact(params.K[0]) << "\n";
        yaml << "      fy: " << exact(params.K[1]) << "\n";
        yaml << "      cx: " << exact(params.K[2]) << "\n";
        yaml << "      cy: " << exact(params.K[3]) << "\n";
        yaml << "      skew: " << exact(params.K[4]) << "\n";
        for (int d = 0; d < 4; ++d) {
            yaml << "      d" << (d + 1) << ": " << exact(params.d[d]) << "\n";
        }
        yaml << "      s_fx: " << exact(params.s_K[0]) << "\n";
        yaml << "      s_fy: " << exact(params.s_K[1]) << "\n";
        yaml << "      s_cx: " << exact(params.s_K[2]) << "\n";
        yaml << "      s_cy: " << exact(params.s_K[3]) << "\n";
        yaml << "      s_skew: " << exact(params.s_K[4]) << "\n";
        for (int d = 0; d < 4; ++d) {
            yaml << "      s_d" << (d + 1) << ": " << exact(params.s_d[d]) << "\n";
        }
        yaml << "    uncertainties_from_mono_stage: "
             << boolean(camera.uncertainties_from_mono_stage) << "\n";
        yaml << "    n_frames: " << camera.result.n_frames << "\n";
        yaml << "    n_frames_heldout: " << camera.result.n_frames_heldout << "\n";
        yaml << "    reprojection_error: " << exact(camera.result.reprojection_error) << "\n";
        yaml << "    fit_seconds: " << exact(camera.result.fit_seconds) << "\n";
        yaml << "    camera_info_path: " << quote(camera.result.camera_info_path) << "\n";
        yaml << "    raw_output_directory: "
             << quote(camera.result.raw_output_directory) << "\n";
        yaml << "    check:\n";
        write_check_yaml(yaml, camera.result.check);
    }

    const std::string rig_path = run_directory_ + "/rig_calibration.yaml";
    if (!write_text_file_atomic(rig_path, yaml.str(), &error)) {
        // This file is the record of everything above; a run without it is not
        // a usable artifact even if the .dwecal was written.
        out.success = false;
        artifacts_ok = false;
        artifact_error = error;
        out.message += "; writing rig_calibration.yaml failed: " + error;
    }

    out.timings.artifacts = artifacts_clock.stop();
    return out;
}

}  // namespace discocal_ros
