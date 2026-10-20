#include "discocal_ros/cross_camera_check.h"

#include <cmath>
#include <sstream>
#include <vector>

#include "discocal_ros/board_frame.h"

namespace discocal_ros {

const double kPoseSanityRmsPx = 5.0;

namespace {

// Sum of squared pixel residuals between a projected board and a detection, over
// the dots of one frame. Summed rather than rooted so the caller can pool
// several frames without weighting them by count.
double frame_sum_sq(const BoardProjector& projector,
                    const std::vector<Eigen::Vector3d>& lattice, double radius,
                    const Eigen::Matrix4d& T, const ::Params& params, int mode,
                    const std::vector<::Shape>& observed) {
    double sum_sq = 0.0;
    for (std::size_t k = 0; k < lattice.size(); ++k) {
        const Eigen::Vector2d predicted =
            projector.project(lattice[k].x(), lattice[k].y(), radius, T, params, mode);
        const double du = predicted.x() - observed[k].x;
        const double dv = predicted.y() - observed[k].y;
        sum_sq += du * du + dv * dv;
    }
    return sum_sq;
}

}  // namespace

CheckResult run_cross_camera_check(const std::vector<HeldOutFrame>& frames,
                                   const std::vector<Eigen::Vector3d>& lattice, double radius,
                                   int n_d, int mode, const ::Params& own_params,
                                   const ::Params& other_params,
                                   const Eigen::Matrix4d& T_own_to_other,
                                   double limit_px) {
    CheckResult result;

    if (frames.empty()) {
        result.reason = "no held-out frames";
        return result;
    }
    if (lattice.empty()) {
        result.reason = "board lattice is empty";
        return result;
    }
    const double n_dots = static_cast<double>(lattice.size());

    const BoardProjector projector(n_d);

    double sum_sq = 0.0;
    int points = 0;

    for (std::size_t f = 0; f < frames.size(); ++f) {
        const HeldOutFrame& frame = frames[f];

        // A frame the other camera did not detect has nothing to compare
        // against, and one whose own detections are the wrong count cannot be
        // indexed against the lattice.
        if (frame.other_shapes.size() != lattice.size() ||
            frame.own_shapes.size() != lattice.size()) {
            ++result.pairs_unpaired;
            continue;
        }

        // Guard first: if the pose does not reproduce the very frame it was
        // solved from, it must not be allowed to judge the extrinsic.
        const double own_rms = std::sqrt(
            frame_sum_sq(projector, lattice, radius, frame.T, own_params, mode, frame.own_shapes) /
            n_dots);
        if (!(own_rms <= kPoseSanityRmsPx)) {
            ++result.pairs_dropped;
            continue;
        }

        // Fold the extrinsic in and project into the other camera. The board
        // frame is still the same board frame, so the lattice indices still line
        // up with the other camera's detections.
        const double pair_sum_sq =
            frame_sum_sq(projector, lattice, radius, T_own_to_other * frame.T, other_params, mode,
                         frame.other_shapes);
        const double pair_rms = std::sqrt(pair_sum_sq / n_dots);

        if (pair_rms > result.worst_pair_rms_px) result.worst_pair_rms_px = pair_rms;
        sum_sq += pair_sum_sq;
        points += static_cast<int>(lattice.size());
        ++result.pairs;
    }

    if (result.pairs == 0) {
        std::ostringstream oss;
        oss << "no usable held-out pairs (" << result.pairs_unpaired << " unpaired, "
            << result.pairs_dropped << " with an unusable pose)";
        result.reason = oss.str();
        return result;
    }

    result.points = points;
    result.rms_px = std::sqrt(sum_sq / static_cast<double>(points));
    result.limit_px = limit_px;
    result.ran = true;
    result.passed = result.rms_px <= limit_px;
    if (!result.passed) {
        std::ostringstream oss;
        oss << "held-out cross-camera RMS " << result.rms_px << " px over " << result.pairs
            << " frame(s) exceeds " << limit_px << " px";
        if (result.pairs_dropped > 0) {
            oss << "; " << result.pairs_dropped << " pair(s) also dropped for an unusable pose";
        }
        result.reason = oss.str();
    }
    return result;
}

}  // namespace discocal_ros
