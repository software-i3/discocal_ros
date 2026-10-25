#include "discocal_ros/known_pose_solver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "ceres/ceres.h"

namespace discocal_ros {
namespace {

std::string format_double(double value, int precision = 6) {
    std::ostringstream oss;
    oss.precision(precision);
    oss << value;
    return oss.str();
}

// The same whitening the vendored CalibrationFunctor applies (CFunctors.h):
// residual = L^T (observed - projected), with L the lower Cholesky factor of the
// detector's inverse 2x2 covariance. A dot with no usable covariance keeps the
// unweighted form, which is what the vendor's use_weight=false path does.
void residual_weights(const ::Shape& shape, bool use_weight, double* c1, double* c2,
                      double* c3) {
    *c1 = 1.0;
    *c2 = 0.0;
    *c3 = 1.0;
    if (!use_weight) return;

    Eigen::Matrix2d covariance;
    covariance << shape.Kxx, shape.Kxy, shape.Kxy, shape.Kyy;
    if (!covariance.allFinite() || covariance.determinant() == 0.0) return;

    const Eigen::LLT<Eigen::Matrix2d> llt(covariance.inverse());
    if (llt.info() != Eigen::Success) return;
    const Eigen::Matrix2d L = llt.matrixL();
    if (!L.allFinite()) return;

    *c1 = L(0, 0);
    *c2 = L(1, 0);
    *c3 = L(1, 1);
}

// K/d cost with every per-frame board pose held fixed. This is the vendor's
// CalibrationFunctor with rot/trans replaced by the externally measured pose;
// the projection path and residual weighting are unchanged, so the residuals
// are in the same units as the image-only solve's.
class FixedPoseFunctor {
public:
    FixedPoseFunctor(const BoardProjector& projector,
                     const std::vector<Eigen::Vector3d>& lattice,
                     const std::vector<::Shape>& observed, const Eigen::Matrix4d& T,
                     double radius, int mode, bool use_weight)
        : projector_(projector),
          lattice_(lattice),
          observed_(observed),
          T_(T),
          radius_(radius),
          mode_(mode),
          weights_(observed.size()) {
        for (std::size_t i = 0; i < observed.size(); ++i) {
            residual_weights(observed[i], use_weight, &weights_[i].c1, &weights_[i].c2,
                             &weights_[i].c3);
        }
    }

    template <typename T>
    bool operator()(const T* const fcs, const T* const distortion, T* residual) const {
        const ::Params params(static_cast<double>(fcs[0]), static_cast<double>(fcs[1]),
                              static_cast<double>(fcs[2]), static_cast<double>(fcs[3]),
                              static_cast<double>(fcs[4]), static_cast<double>(distortion[0]),
                              static_cast<double>(distortion[1]),
                              static_cast<double>(distortion[2]),
                              static_cast<double>(distortion[3]));

        for (std::size_t i = 0; i < observed_.size(); ++i) {
            const Eigen::Vector2d projected =
                projector_.project(lattice_[i].x(), lattice_[i].y(), radius_, T_, params, mode_);
            const double du = observed_[i].x - projected.x();
            const double dv = observed_[i].y - projected.y();
            residual[2 * i] = static_cast<T>(weights_[i].c1 * du + weights_[i].c2 * dv);
            residual[2 * i + 1] = static_cast<T>(weights_[i].c3 * dv);
        }
        return true;
    }

private:
    struct Weights {
        double c1 = 1.0;
        double c2 = 0.0;
        double c3 = 1.0;
    };

    const BoardProjector& projector_;
    const std::vector<Eigen::Vector3d>& lattice_;
    const std::vector<::Shape>& observed_;
    Eigen::Matrix4d T_;
    double radius_;
    int mode_;
    std::vector<Weights> weights_;
};

bool finite_value(double value) { return std::isfinite(value); }

}  // namespace

KnownPoseSolver::KnownPoseSolver(const BoardConfig& board, const KnownPoseOptions& options)
    : board_(board),
      options_(options),
      lattice_(board_lattice(board)),
      symmetries_(lattice_symmetries(lattice_)),
      projector_(new BoardProjector(board.n_d)) {}

KnownPoseSolver::~KnownPoseSolver() {}

bool KnownPoseSolver::valid_params(const ::Params& params) const {
    for (int i = 0; i < 5; ++i) {
        if (!finite_value(params.K[i])) return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (!finite_value(params.d[i])) return false;
    }
    if (!(params.fx() > 0.0) || !(params.fy() > 0.0)) return false;

    // A focal ratio far from one is not representable by this model in any
    // useful way, and it is the usual signature of a degenerate linear fit.
    const double ratio = params.fx() / params.fy();
    if (!(ratio > 0.2 && ratio < 5.0)) return false;

    if (options_.image_width > 0 && options_.image_height > 0) {
        const double max_dimension =
            static_cast<double>(std::max(options_.image_width, options_.image_height));
        if (!(params.fx() > 0.05 * max_dimension && params.fx() < 20.0 * max_dimension)) {
            return false;
        }
        if (!(params.fy() > 0.05 * max_dimension && params.fy() < 20.0 * max_dimension)) {
            return false;
        }
        if (!(params.cx() > -0.5 * options_.image_width &&
              params.cx() < 1.5 * options_.image_width)) {
            return false;
        }
        if (!(params.cy() > -0.5 * options_.image_height &&
              params.cy() < 1.5 * options_.image_height)) {
            return false;
        }
    }
    return true;
}

std::vector<Eigen::Vector3d> KnownPoseSolver::reorder(
    const std::vector<Eigen::Vector3d>& points, const std::vector<int>& permutation) const {
    std::vector<Eigen::Vector3d> out(permutation.size());
    for (std::size_t k = 0; k < permutation.size(); ++k) {
        out[k] = points[static_cast<std::size_t>(permutation[k])];
    }
    return out;
}

double KnownPoseSolver::projection_rms_px(const std::vector<::Shape>& observed,
                                          const Eigen::Matrix4d& T,
                                          const ::Params& params) const {
    if (observed.size() != lattice_.size() || observed.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const std::vector<Eigen::Vector2d> projected = projector_->project_all(
        lattice_, board_.radius, T, params, options_.projection_mode);
    double squared_error = 0.0;
    for (std::size_t i = 0; i < observed.size(); ++i) {
        const double du = projected[i].x() - observed[i].x;
        const double dv = projected[i].y() - observed[i].y;
        if (!std::isfinite(du) || !std::isfinite(dv)) {
            return std::numeric_limits<double>::infinity();
        }
        squared_error += du * du + dv * dv;
    }
    return std::sqrt(squared_error / static_cast<double>(observed.size()));
}

bool KnownPoseSolver::solve_linear_k(const std::vector<Eigen::Vector3d>& points,
                                     const std::vector<::Shape>& observed,
                                     ::Params* out) const {
    if (out == NULL || points.size() != observed.size() || points.size() < 3) return false;

    // u = (fx X + skew Y + cx Z)/Z and v = (fy Y + cy Z)/Z. Dividing through by
    // Z is not cosmetic: the raw rows have entries of order Z while the pixel
    // coordinates are of order Z*f, so the un-normalized system is much worse
    // conditioned (and skew, whose column is Y, is the first to suffer).
    const int n = static_cast<int>(points.size());
    Eigen::MatrixXd A(2 * n, 5);
    Eigen::VectorXd b(2 * n);
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector3d& p = points[static_cast<std::size_t>(i)];
        if (!p.allFinite() || !(p.z() > 0.0)) return false;
        const double x = p.x() / p.z();
        const double y = p.y() / p.z();
        A.row(2 * i) << x, y, 1.0, 0.0, 0.0;
        b(2 * i) = observed[static_cast<std::size_t>(i)].x;
        A.row(2 * i + 1) << 0.0, 0.0, 0.0, y, 1.0;
        b(2 * i + 1) = observed[static_cast<std::size_t>(i)].y;
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& singular = svd.singularValues();
    if (singular.size() < 5 || !(singular(0) > 0.0)) return false;
    if (singular(4) <= singular(0) * 1e-10) return false;

    const Eigen::VectorXd x = svd.solve(b);
    if (!x.allFinite()) return false;

    // x = [fx, skew, cx, fy, cy]; Params is (fx, fy, cx, cy, skew).
    ::Params params(x(0), x(3), x(2), x(4), x(1));
    if (!valid_params(params)) return false;

    *out = params;
    return true;
}

bool KnownPoseSolver::linear_bootstrap(const std::vector<KnownPoseFrame>& frames,
                                       ::Params* out) const {
    if (out == NULL) return false;

    struct Pick {
        bool ok = false;
        int symmetry = -1;
        RigidFit fit;
        ::Params params;
        double score = std::numeric_limits<double>::infinity();
    };
    std::vector<Pick> picks(frames.size());

    for (std::size_t f = 0; f < frames.size(); ++f) {
        const KnownPoseFrame& frame = frames[f];
        if (frame.points.size() != lattice_.size() ||
            frame.shapes.size() != lattice_.size()) {
            continue;
        }

        // The label-order cloud is the rigid gate. For a symmetric board its
        // residual is the same under every symmetry permutation, so a failure
        // here cannot be fixed by relabeling.
        const RigidFit gate = fit_rigid_transform(lattice_, frame.points);
        if (!gate.ok || !(gate.rms <= options_.external.max_rigidity_rms_m)) continue;

        for (std::size_t s = 0; s < symmetries_.size(); ++s) {
            const LatticeSymmetry& symmetry = symmetries_[s];
            if (symmetry.permutation.size() != lattice_.size()) continue;

            std::vector<Eigen::Vector3d> ordered;
            bool valid_permutation = true;
            ordered.resize(lattice_.size());
            for (std::size_t k = 0; k < lattice_.size(); ++k) {
                const int label = symmetry.permutation[k];
                if (label < 0 || static_cast<std::size_t>(label) >= frame.points.size()) {
                    valid_permutation = false;
                    break;
                }
                ordered[k] = frame.points[static_cast<std::size_t>(label)];
            }
            if (!valid_permutation) continue;

            const RigidFit fit = fit_rigid_transform(lattice_, ordered);
            if (!fit.ok) continue;

            ::Params linear;
            if (!solve_linear_k(ordered, frame.shapes, &linear)) continue;

            const double score = projection_rms_px(frame.shapes, fit.T, linear);
            if (score < picks[f].score) {
                picks[f].ok = true;
                picks[f].symmetry = static_cast<int>(s);
                picks[f].fit = fit;
                picks[f].params = linear;
                picks[f].score = score;
            }
        }
    }

    std::vector<int> selected;
    for (std::size_t f = 0; f < picks.size(); ++f) {
        if (picks[f].ok && picks[f].score <= options_.external.max_match_rms_px) {
            selected.push_back(static_cast<int>(f));
        }
    }
    if (selected.empty()) {
        for (std::size_t f = 0; f < picks.size(); ++f) {
            if (picks[f].ok) selected.push_back(static_cast<int>(f));
        }
    }
    if (selected.empty()) return false;

    // One global K from every frame whose own symmetry choice was consistent
    // with its detection. Concatenating the correspondences is valid because
    // the equations are independent per dot.
    std::vector<Eigen::Vector3d> all_points;
    std::vector<::Shape> all_shapes;
    for (std::size_t i = 0; i < selected.size(); ++i) {
        const int f = selected[i];
        const LatticeSymmetry& symmetry = symmetries_[static_cast<std::size_t>(picks[f].symmetry)];
        const std::vector<Eigen::Vector3d> ordered = reorder(frames[f].points, symmetry.permutation);
        all_points.insert(all_points.end(), ordered.begin(), ordered.end());
        all_shapes.insert(all_shapes.end(), frames[f].shapes.begin(), frames[f].shapes.end());
    }

    ::Params global;
    if (solve_linear_k(all_points, all_shapes, &global)) {
        *out = global;
        return true;
    }

    // The combined system was rank-deficient (e.g. every selected frame sees
    // the board from nearly the same direction). The best single-frame estimate
    // is still a usable starting point.
    int best = -1;
    for (std::size_t f = 0; f < picks.size(); ++f) {
        if (!picks[f].ok) continue;
        if (best < 0 || picks[f].score < picks[best].score) best = static_cast<int>(f);
    }
    if (best < 0) return false;
    *out = picks[best].params;
    return true;
}

KnownPoseSolver::MatchSet KnownPoseSolver::match_frames(
    const std::vector<KnownPoseFrame>& frames, const ::Params& params,
    const std::vector<std::vector<int> >& previous) const {
    MatchSet set;
    set.matches.resize(frames.size());
    set.permutations.resize(frames.size());
    set.used.resize(frames.size(), false);

    BoardMatchOptions options;
    options.max_rigidity_rms_m = options_.external.max_rigidity_rms_m;
    options.max_match_rms_px = options_.external.max_match_rms_px;
    options.min_match_margin_px = options_.external.min_match_margin_px;
    options.temporal_margin_px = options_.external.temporal_margin_px;
    options.allow_ambiguous = (options_.external.ambiguous_policy == "use_best");
    options.projection_mode = options_.projection_mode;

    for (std::size_t i = 0; i < frames.size(); ++i) {
        const std::vector<int>* previous_permutation = NULL;
        if (i < previous.size() && !previous[i].empty()) {
            previous_permutation = &previous[i];
        }

        const BoardMatch match = match_board_pose(
            lattice_, frames[i].points, frames[i].shapes, params, board_.radius, *projector_,
            options, symmetries_, previous_permutation);
        set.matches[i] = match;

        // Keep the permutation even when the policy rejects the frame: an
        // ambiguous frame may become usable on the next EM pass once the
        // intrinsics have moved, and continuity is what resolves it.
        if (match.permutation.size() == lattice_.size()) {
            set.permutations[i] = match.permutation;
        }
        if (match.ok) {
            set.used[i] = true;
            ++set.used_count;
        }
    }
    return set;
}

bool KnownPoseSolver::solve_fixed(const std::vector<KnownPoseFrame>& frames,
                                  const MatchSet& set, ::Params* params,
                                  std::string* reason) const {
    if (params == NULL) return false;

    const int n_d = std::max(0, std::min(4, board_.n_d));
    // The projection model consumes d1..d_n_d only (MomentsTracker's
    // distort_Point sums max_dim+1 terms). Zero the rest so the reported
    // calibration does not carry values that had no effect on the fit.
    for (int i = n_d; i < 4; ++i) params->d[i] = 0.0;

    ceres::Problem problem;
    int blocks = 0;
    for (std::size_t i = 0; i < frames.size() && i < set.used.size(); ++i) {
        if (!set.used[i]) continue;
        if (frames[i].shapes.size() != lattice_.size()) continue;

        const int num_residuals = 2 * static_cast<int>(frames[i].shapes.size());
        ceres::CostFunction* cost =
            new ceres::NumericDiffCostFunction<FixedPoseFunctor, ceres::CENTRAL,
                                               ceres::DYNAMIC, 5, 4>(
                new FixedPoseFunctor(*projector_, lattice_, frames[i].shapes,
                                     set.matches[i].T, board_.radius,
                                     options_.projection_mode,
                                     options_.external.use_covariance_weights),
                ceres::TAKE_OWNERSHIP, num_residuals);
        problem.AddResidualBlock(cost, new ceres::TrivialLoss(), params->K, params->d);
        ++blocks;
    }

    if (blocks == 0) {
        if (reason != NULL) *reason = "no matched frame could enter the solve";
        return false;
    }

    if (n_d == 0) {
        problem.SetParameterBlockConstant(params->d);
    } else if (n_d < 4) {
        std::vector<int> constant_indices;
        for (int i = n_d; i < 4; ++i) constant_indices.push_back(i);
        problem.SetParameterization(params->d,
                                    new ceres::SubsetParameterization(4, constant_indices));
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = std::max(1, options_.ceres_threads);
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type == ceres::FAILURE || !std::isfinite(summary.final_cost)) {
        if (reason != NULL) *reason = "the fixed-pose Ceres solve failed";
        return false;
    }
    if (!valid_params(*params)) {
        if (reason != NULL) *reason = "the fixed-pose solve produced invalid intrinsics";
        return false;
    }

    // --- uncertainty ---------------------------------------------------------
    // The residuals are already whitened by the per-dot covariance, so J^T J is
    // the same information matrix the vendor forms as J^T M_cov^-1 J. Only the
    // first 5+n_d columns are identifiable; the frozen distortion coefficients
    // have zero columns and are excluded.
    ceres::Problem::EvaluateOptions eval_options;
    eval_options.apply_loss_function = false;
    double cost = 0.0;
    std::vector<double> residuals;
    std::vector<double> gradient;
    ceres::CRSMatrix jacobian;
    std::array<double, 9> sigmas = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    const int n_keep = 5 + n_d;
    if (problem.Evaluate(eval_options, &cost, &residuals, &gradient, &jacobian) &&
        jacobian.num_cols >= n_keep) {
        Eigen::MatrixXd J =
            Eigen::MatrixXd::Zero(jacobian.num_rows, jacobian.num_cols);
        for (int row = 0; row < jacobian.num_rows; ++row) {
            for (int p = jacobian.rows[row]; p < jacobian.rows[row + 1]; ++p) {
                J(row, jacobian.cols[p]) = jacobian.values[p];
            }
        }
        const Eigen::MatrixXd H =
            J.leftCols(n_keep).transpose() * J.leftCols(n_keep);
        const Eigen::MatrixXd covariance = H.inverse();
        if (covariance.allFinite()) {
            for (int i = 0; i < n_keep; ++i) {
                sigmas[static_cast<std::size_t>(i)] =
                    std::sqrt(std::max(0.0, covariance(i, i)));
            }
        }
    }
    params->update_unc(sigmas);

    return true;
}

bool KnownPoseSolver::solve(const std::vector<KnownPoseFrame>& frames,
                            const ::Params* initial_params, KnownPoseResult* out) {
    if (out == NULL) return false;
    *out = KnownPoseResult();
    out->frames.resize(frames.size());

    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    if (static_cast<int>(frames.size()) < options_.min_frames) {
        std::ostringstream oss;
        oss << "only " << frames.size() << " frame(s) reached the external solve; at least "
            << options_.min_frames << " are needed";
        out->message = oss.str();
        return false;
    }

    // Rigidity of the external measurements themselves, independent of K.
    double rigidity_sum = 0.0;
    int rigidity_count = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const RigidFit fit = fit_rigid_transform(lattice_, frames[i].points);
        if (fit.ok) {
            rigidity_sum += fit.rms;
            ++rigidity_count;
        }
    }
    if (rigidity_count > 0) {
        out->mean_rigidity_rms_m = rigidity_sum / static_cast<double>(rigidity_count);
    }

    // --- initialization candidates ------------------------------------------
    std::vector<InitCandidate> candidates;
    if (initial_params != NULL && valid_params(*initial_params)) {
        InitCandidate candidate;
        candidate.params = *initial_params;
        candidate.label = "config";
        candidates.push_back(candidate);
    }
    if (options_.external.use_dlt_initialization) {
        ::Params linear;
        if (linear_bootstrap(frames, &linear)) {
            InitCandidate candidate;
            candidate.params = linear;
            candidate.label = "linear";
            candidates.push_back(candidate);
        }
    }
    if (options_.image_width > 0 && options_.image_height > 0) {
        const double max_dimension =
            static_cast<double>(std::max(options_.image_width, options_.image_height));
        const double factors[] = {0.5, 0.75, 1.0, 1.25, 1.5, 2.0};
        for (std::size_t i = 0; i < sizeof(factors) / sizeof(factors[0]); ++i) {
            const double focal = factors[i] * max_dimension;
            InitCandidate candidate;
            candidate.params = ::Params(focal, focal, 0.5 * options_.image_width,
                                        0.5 * options_.image_height, 0.0);
            std::ostringstream label;
            label << "focal " << factors[i] << "x";
            candidate.label = label.str();
            candidates.push_back(candidate);
        }
    }

    if (candidates.empty()) {
        out->message =
            "no usable intrinsic initialization: no config intrinsics, the linear estimate "
            "failed, and the image size is unknown so no focal grid could be tried";
        return false;
    }

    // --- pick the starting point --------------------------------------------
    const std::vector<std::vector<int> > no_previous;
    const double penalty = std::max(options_.external.max_match_rms_px, 1e-3);

    ::Params params;
    std::string initialization;
    MatchSet current;
    bool have_candidate = false;
    double best_cost = std::numeric_limits<double>::infinity();
    int best_used = -1;

    for (std::size_t c = 0; c < candidates.size(); ++c) {
        const MatchSet trial = match_frames(frames, candidates[c].params, no_previous);
        if (trial.used_count < options_.min_frames) continue;

        double total = 0.0;
        for (std::size_t i = 0; i < trial.matches.size(); ++i) {
            const double score = trial.matches[i].match_rms_px;
            if (!std::isfinite(score) || score > 1e299) {
                total += penalty;
            } else {
                total += std::min(score, penalty);
            }
        }
        const double cost = total / static_cast<double>(frames.size());

        if (!have_candidate || cost < best_cost - 1e-12 ||
            (std::fabs(cost - best_cost) <= 1e-12 && trial.used_count > best_used)) {
            have_candidate = true;
            best_cost = cost;
            best_used = trial.used_count;
            params = candidates[c].params;
            initialization = candidates[c].label;
            current = trial;
        }
    }

    if (!have_candidate) {
        std::ostringstream oss;
        oss << "none of the " << candidates.size()
            << " intrinsic initialization candidate(s) matched at least " << options_.min_frames
            << " frame(s); check the pose topic, the time synchronization and the board geometry";
        out->message = oss.str();
        out->initialization = candidates.empty() ? "" : candidates.front().label;
        out->fit_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        return false;
    }
    out->initialization = initialization;

    // --- EM: match, solve, rematch ------------------------------------------
    const int max_iterations = std::max(1, options_.external.max_em_iterations);
    bool converged = false;
    std::string solve_error;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        out->em_iterations = iteration + 1;

        if (current.used_count < options_.min_frames) {
            std::ostringstream oss;
            oss << "only " << current.used_count << " frame(s) matched in EM iteration "
                << (iteration + 1) << "; at least " << options_.min_frames << " are needed";
            out->message = oss.str();
            out->params = params;
            out->converged = false;
            out->fit_seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            return false;
        }

        ::Params solved = params;
        if (!solve_fixed(frames, current, &solved, &solve_error)) {
            out->message = "EM iteration " + std::to_string(iteration + 1) + ": " + solve_error;
            out->params = params;
            out->converged = false;
            out->fit_seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            return false;
        }
        params = solved;

        MatchSet next = match_frames(frames, params, current.permutations);
        const bool same_pairing =
            next.used == current.used && next.permutations == current.permutations;
        current = next;
        if (same_pairing) {
            converged = true;
            break;
        }
    }

    // If the loop stopped because the pairing was still moving, the parameters
    // come from the previous pairing while `current` is the new one. Solve once
    // more so the reported K and the reported matches describe the same data.
    if (!converged && current.used_count >= options_.min_frames) {
        ::Params solved = params;
        if (solve_fixed(frames, current, &solved, &solve_error)) params = solved;
    }

    // --- result --------------------------------------------------------------
    out->params = params;
    out->converged = converged;

    int matched = 0;
    int ambiguous = 0;
    int rejected = 0;
    std::vector<Eigen::Matrix4d> pose_list;
    std::vector<std::vector<::Shape> > detection_list;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        out->frames[i].match = current.matches[i];
        out->frames[i].used = current.used[i];
        if (current.matches[i].ambiguous) ++ambiguous;
        if (current.used[i]) {
            ++matched;
            pose_list.push_back(current.matches[i].T);
            detection_list.push_back(frames[i].shapes);
        } else {
            ++rejected;
        }
    }
    out->matched_frames = matched;
    out->ambiguous_frames = ambiguous;
    out->rejected_frames = rejected;

    out->reprojection_error_px = projector_->reprojection_error(
        pose_list, detection_list, lattice_, board_.radius, params,
        options_.projection_mode);

    out->fit_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    if (matched < options_.min_frames) {
        std::ostringstream oss;
        oss << "the final pairing used only " << matched << " frame(s); at least "
            << options_.min_frames << " are needed";
        out->message = oss.str();
        return false;
    }
    if (!valid_params(params)) {
        out->message = "the external solve produced invalid intrinsics";
        return false;
    }

    std::ostringstream summary;
    summary << "external-pose solve: " << matched << " matched frame(s), " << ambiguous
            << " ambiguous, " << rejected << " rejected; initialized from " << initialization;
    out->message = summary.str();
    out->ok = true;
    return true;
}

}  // namespace discocal_ros
