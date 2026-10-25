#include "discocal_ros/external_points.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace discocal_ros {
namespace {

struct SymmetryMatrix {
    const char* name;
    // x' = a*x + b*y, y' = c*x + d*y, about the point set's centroid.
    double a, b, c, d;
};

const SymmetryMatrix kSymmetryMatrices[] = {
    {"identity", 1.0, 0.0, 0.0, 1.0},
    {"rot90", 0.0, -1.0, 1.0, 0.0},
    {"rot180", -1.0, 0.0, 0.0, -1.0},
    {"rot270", 0.0, 1.0, -1.0, 0.0},
    {"flip_x", -1.0, 0.0, 0.0, 1.0},
    {"flip_y", 1.0, 0.0, 0.0, -1.0},
    {"transpose", 0.0, 1.0, 1.0, 0.0},
    {"anti_transpose", 0.0, -1.0, -1.0, 0.0},
};

double min_spacing(const std::vector<Eigen::Vector3d>& lattice) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < lattice.size(); ++i) {
        for (std::size_t j = i + 1; j < lattice.size(); ++j) {
            const double d = (lattice[i] - lattice[j]).norm();
            if (d > 1e-12 && d < best) best = d;
        }
    }
    return best;
}

std::string format_double(double value, int precision = 6) {
    std::ostringstream oss;
    oss.precision(precision);
    oss << value;
    return oss.str();
}

bool finite_point(const Eigen::Vector3d& p) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}

}  // namespace

std::vector<LatticeSymmetry> lattice_symmetries(
    const std::vector<Eigen::Vector3d>& lattice) {
    std::vector<LatticeSymmetry> out;
    if (lattice.size() < 2) {
        LatticeSymmetry identity;
        identity.index = 0;
        identity.name = "identity";
        identity.permutation.resize(lattice.size());
        for (std::size_t i = 0; i < lattice.size(); ++i) {
            identity.permutation[i] = static_cast<int>(i);
        }
        out.push_back(identity);
        return out;
    }

    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < lattice.size(); ++i) {
        center += lattice[i].head<2>();
    }
    center /= static_cast<double>(lattice.size());

    const double spacing = min_spacing(lattice);
    const double tolerance = std::max(1e-9, spacing * 1e-6);
    const double tolerance_sq = tolerance * tolerance;

    std::set<std::vector<int> > seen;
    for (std::size_t m = 0; m < sizeof(kSymmetryMatrices) / sizeof(kSymmetryMatrices[0]);
         ++m) {
        const SymmetryMatrix& matrix = kSymmetryMatrices[m];
        std::vector<int> permutation(lattice.size(), -1);
        std::vector<bool> used(lattice.size(), false);
        bool valid = true;

        for (std::size_t k = 0; k < lattice.size() && valid; ++k) {
            const Eigen::Vector2d p = lattice[k].head<2>() - center;
            const Eigen::Vector2d q(matrix.a * p.x() + matrix.b * p.y(),
                                    matrix.c * p.x() + matrix.d * p.y());
            const Eigen::Vector2d target = q + center;

            int best_index = -1;
            double best_distance = std::numeric_limits<double>::infinity();
            for (std::size_t j = 0; j < lattice.size(); ++j) {
                const double distance = (lattice[j].head<2>() - target).squaredNorm();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index = static_cast<int>(j);
                }
            }
            if (best_index < 0 || best_distance > tolerance_sq || used[best_index]) {
                valid = false;
                break;
            }
            used[best_index] = true;
            permutation[k] = best_index;
        }

        if (!valid) continue;
        if (seen.find(permutation) != seen.end()) continue;
        seen.insert(permutation);

        LatticeSymmetry symmetry;
        symmetry.index = static_cast<int>(out.size());
        symmetry.name = matrix.name;
        symmetry.permutation = permutation;
        out.push_back(symmetry);
    }

    // The identity always validates, but keep the invariant explicit: an empty
    // result would make every frame unmatchable for a reason that is hard to
    // see from the caller.
    if (out.empty()) {
        LatticeSymmetry identity;
        identity.index = 0;
        identity.name = "identity";
        identity.permutation.resize(lattice.size());
        for (std::size_t i = 0; i < lattice.size(); ++i) {
            identity.permutation[i] = static_cast<int>(i);
        }
        out.push_back(identity);
    }
    return out;
}

RigidFit fit_rigid_transform(const std::vector<Eigen::Vector3d>& model,
                             const std::vector<Eigen::Vector3d>& observed) {
    RigidFit fit;
    if (model.size() != observed.size()) {
        fit.reason = "model and observed point counts differ";
        return fit;
    }
    if (model.size() < 3) {
        fit.reason = "a rigid transform needs at least three points";
        return fit;
    }

    Eigen::Vector3d model_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d observed_center = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < model.size(); ++i) {
        if (!finite_point(model[i]) || !finite_point(observed[i])) {
            fit.reason = "a point is not finite";
            return fit;
        }
        model_center += model[i];
        observed_center += observed[i];
    }
    model_center /= static_cast<double>(model.size());
    observed_center /= static_cast<double>(observed.size());

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < model.size(); ++i) {
        H += (model[i] - model_center) * (observed[i] - observed_center).transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
        D(2, 2) = -1.0;
    }
    const Eigen::Matrix3d R = svd.matrixV() * D * svd.matrixU().transpose();
    if (!R.allFinite()) {
        fit.reason = "the rigid fit produced a non-finite rotation";
        return fit;
    }

    const Eigen::Vector3d t = observed_center - R * model_center;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;

    double squared_error = 0.0;
    for (std::size_t i = 0; i < model.size(); ++i) {
        squared_error += (R * model[i] + t - observed[i]).squaredNorm();
    }

    fit.ok = true;
    fit.T = T;
    fit.rms = std::sqrt(squared_error / static_cast<double>(model.size()));
    return fit;
}

BoardMatch match_board_pose(const std::vector<Eigen::Vector3d>& lattice,
                            const std::vector<Eigen::Vector3d>& external_points,
                            const std::vector<::Shape>& observed, const ::Params& params,
                            double radius, const BoardProjector& projector,
                            const BoardMatchOptions& options,
                            const std::vector<LatticeSymmetry>& symmetries,
                            const std::vector<int>* previous_permutation) {
    BoardMatch match;
    if (lattice.empty() || external_points.size() != lattice.size() ||
        observed.size() != lattice.size()) {
        match.reason = "lattice, external points and detection have different sizes";
        return match;
    }
    if (symmetries.empty()) {
        match.reason = "no lattice symmetries were supplied";
        return match;
    }

    for (std::size_t i = 0; i < external_points.size(); ++i) {
        if (!finite_point(external_points[i])) {
            match.reason = "an external point is not finite";
            return match;
        }
        if (!(external_points[i].z() > 0.0)) {
            match.reason = "an external point has non-positive z in the optical frame";
            return match;
        }
    }

    // Rigidity is checked with the label order as published. For a symmetric
    // lattice the residual is invariant under the symmetry permutations, so
    // this is the board's rigidity regardless of which label ordering is real.
    const RigidFit rigidity = fit_rigid_transform(lattice, external_points);
    if (!rigidity.ok) {
        match.reason = "rigidity fit failed: " + rigidity.reason;
        return match;
    }
    match.rigidity_rms_m = rigidity.rms;
    if (!(rigidity.rms <= options.max_rigidity_rms_m)) {
        match.reason = "the external point cloud is not a rigid copy of the board: RMS " +
                       format_double(rigidity.rms, 6) + " m > " +
                       format_double(options.max_rigidity_rms_m, 6) + " m";
        return match;
    }

    struct Candidate {
        int symmetry_index = -1;
        const LatticeSymmetry* symmetry = NULL;
        RigidFit fit;
        double score = std::numeric_limits<double>::infinity();
    };

    Candidate best;
    Candidate second;
    Candidate previous;
    bool have_previous = false;
    const bool have_previous_permutation =
        previous_permutation != NULL && previous_permutation->size() == lattice.size();

    std::vector<Eigen::Vector3d> reordered(lattice.size());
    for (std::size_t s = 0; s < symmetries.size(); ++s) {
        const LatticeSymmetry& symmetry = symmetries[s];
        if (symmetry.permutation.size() != lattice.size()) continue;

        bool valid_permutation = true;
        for (std::size_t k = 0; k < lattice.size(); ++k) {
            const int label = symmetry.permutation[k];
            if (label < 0 || static_cast<std::size_t>(label) >= external_points.size()) {
                valid_permutation = false;
                break;
            }
            reordered[k] = external_points[static_cast<std::size_t>(label)];
        }
        if (!valid_permutation) continue;

        const RigidFit fit = fit_rigid_transform(lattice, reordered);
        if (!fit.ok) continue;

        const std::vector<Eigen::Vector2d> projected =
            projector.project_all(lattice, radius, fit.T, params, options.projection_mode);
        if (projected.size() != observed.size()) continue;

        double squared_error = 0.0;
        bool score_ok = true;
        for (std::size_t k = 0; k < observed.size(); ++k) {
            const double du = projected[k].x() - observed[k].x;
            const double dv = projected[k].y() - observed[k].y;
            if (!std::isfinite(du) || !std::isfinite(dv)) {
                score_ok = false;
                break;
            }
            squared_error += du * du + dv * dv;
        }
        if (!score_ok) continue;

        Candidate candidate;
        candidate.symmetry_index = symmetry.index;
        candidate.symmetry = &symmetry;
        candidate.fit = fit;
        candidate.score = std::sqrt(squared_error / static_cast<double>(observed.size()));

        if (candidate.score < best.score) {
            second = best;
            best = candidate;
        } else if (candidate.score < second.score) {
            second = candidate;
        }

        if (have_previous_permutation && symmetry.permutation == *previous_permutation) {
            previous = candidate;
            have_previous = true;
        }
    }

    if (best.symmetry == NULL) {
        match.reason = "no symmetry candidate produced a usable projection";
        return match;
    }

    Candidate chosen = best;
    if (have_previous && options.temporal_margin_px > 0.0 &&
        previous.score <= best.score + options.temporal_margin_px) {
        chosen = previous;
        match.resolved_by_continuity = (previous.symmetry_index != best.symmetry_index);
    }

    // Every reported figure describes the candidate actually used: the gate
    // below must not pass a continuity choice on the strength of a different
    // symmetry's score. The runner-up is the best candidate other than the
    // chosen one, so a continuity choice that scored slightly worse than the
    // best reports the (possibly negative) gap that the preference bridged.
    const Candidate runner_up =
        (chosen.symmetry_index == best.symmetry_index) ? second : best;

    match.symmetry_index = chosen.symmetry_index;
    match.symmetry_name = chosen.symmetry->name;
    match.permutation = chosen.symmetry->permutation;
    match.T = chosen.fit.T;
    match.match_rms_px = chosen.score;
    match.second_best_rms_px =
        runner_up.symmetry == NULL ? std::numeric_limits<double>::infinity() : runner_up.score;
    match.margin_px = match.second_best_rms_px - match.match_rms_px;
    match.ambiguous = !match.resolved_by_continuity &&
                      (match.margin_px < options.min_match_margin_px);

    if (!(chosen.score <= options.max_match_rms_px)) {
        match.reason = "chosen symmetry \"" + chosen.symmetry->name + "\" reprojects at RMS " +
                       format_double(chosen.score, 4) + " px > " +
                       format_double(options.max_match_rms_px, 4) + " px";
        return match;
    }
    if (match.ambiguous && !options.allow_ambiguous) {
        match.reason = "the frame is symmetry-ambiguous: best and second-best differ by " +
                       format_double(match.margin_px, 4) + " px < " +
                       format_double(options.min_match_margin_px, 4) + " px";
        return match;
    }

    match.ok = true;
    return match;
}

}  // namespace discocal_ros
