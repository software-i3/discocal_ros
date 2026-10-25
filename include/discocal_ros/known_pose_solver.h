#ifndef DISCOCAL_ROS_KNOWN_POSE_SOLVER_H_
#define DISCOCAL_ROS_KNOWN_POSE_SOLVER_H_

// Intrinsics from frames whose board pose is measured, not estimated.
//
// The image-only calibrator treats each frame's board pose as an unknown and
// refines K, d and every pose together. Here each frame carries the target dots'
// positions in the camera's optical frame, so the pose is a measurement: a rigid
// fit (Kabsch) from the nominal lattice gives it directly. Only K and d are free.
//
// Two things make that a calibration rather than a curve fit:
//
//   - The pixel/point pairing is unknown. The publisher indexes dots by their
//     original label and the detector orders them by its own lattice walk, so
//     the frames differ by an element of the board's symmetry group. The matcher
//     resolves it per frame (external_points.h) and the EM loop below alternates
//     matching with solving until the permutations stop changing.
//   - K itself is initially unknown, so the first match cannot use it. A linear
//     estimate from the known 3D-2D correspondences, plus a grid of focal
//     candidates, gives the loop a starting point. Neither needs a board pose or
//     Zhang's absolute conic; the points are already in camera frame.
//
// The solve itself is the vendored model: conic/ellipse-centroid projection
// through BoardProjector, per-dot covariance whitening, TrivialLoss, and the
// same K/d parameter layout. That keeps its reprojection error comparable with
// the image-only path's.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "discocal_ros/board_frame.h"
#include "discocal_ros/external_points.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params, ::Shape

namespace discocal_ros {

// One frame's detector output and the externally measured dot positions.
// `points` is in label order (index = original label); its size must equal
// shapes.size(). Order of `frames` is the solve/report order and should be
// chronological so temporal continuity is meaningful.
struct KnownPoseFrame {
    std::int64_t stamp_ns = 0;
    std::vector<::Shape> shapes;
    std::vector<Eigen::Vector3d> points;
};

struct KnownPoseOptions {
    int image_width = 0;
    int image_height = 0;
    // 0 for a circle board, 2 for a square one (BoardProjector's modes).
    int projection_mode = 0;
    // The same floor the image-only path uses: CalibrateSession::min_fit_frames.
    int min_frames = 5;
    int ceres_threads = 1;
    ExternalConfig external;
};

struct KnownPoseFrameResult {
    BoardMatch match;
    // True when this frame entered the final fit. False covers a failed
    // rigidity check, a symmetry ambiguity that the policy skips, and a
    // reprojection score above the threshold -- `match.reason` says which.
    bool used = false;
};

struct KnownPoseResult {
    bool ok = false;
    std::string message;
    ::Params params;

    // One entry per input frame, in the order supplied.
    std::vector<KnownPoseFrameResult> frames;
    int matched_frames = 0;
    int ambiguous_frames = 0;
    int rejected_frames = 0;

    // The initialization the EM loop started from, for the log: "config",
    // "linear", or e.g. "focal 1.25x".
    std::string initialization;
    int em_iterations = 0;
    // True when two consecutive match passes produced the same per-frame
    // permutations.
    bool converged = false;

    // Mean over all frames of the rigidity residual of the label-order Kabsch
    // fit. Independent of K, so it describes the external measurements alone.
    double mean_rigidity_rms_m = 0.0;
    // The library's own metric: mean over frames of the mean dot distance,
    // after the final solve. Comparable with the image-only path's number.
    double reprojection_error_px = 0.0;
    double fit_seconds = 0.0;
};

class KnownPoseSolver {
public:
    KnownPoseSolver(const BoardConfig& board, const KnownPoseOptions& options);
    ~KnownPoseSolver();

    // `initial_params`, when non-null and valid, is one candidate starting
    // point. It is not required: the solver also derives a linear estimate and,
    // when the image size is known, tries a focal grid. All candidates are
    // scored by matching and the best one starts the EM loop.
    //
    // Returns out->ok == false with a message when fewer than min_frames match,
    // no candidate produces a usable K, or the Ceres solve leaves an invalid
    // parameter set.
    bool solve(const std::vector<KnownPoseFrame>& frames, const ::Params* initial_params,
               KnownPoseResult* out);

private:
    KnownPoseSolver(const KnownPoseSolver&);
    KnownPoseSolver& operator=(const KnownPoseSolver&);

    struct MatchSet {
        std::vector<BoardMatch> matches;
        std::vector<std::vector<int> > permutations;  // empty when not used
        std::vector<bool> used;
        int used_count = 0;
    };

    struct InitCandidate {
        ::Params params;
        std::string label;
    };

    bool valid_params(const ::Params& params) const;

    // Reorders external points into detector order: out[k] = points[perm[k]].
    std::vector<Eigen::Vector3d> reorder(const std::vector<Eigen::Vector3d>& points,
                                         const std::vector<int>& permutation) const;

    // One match pass. `previous`, when sized like `frames`, carries the
    // permutations from the previous EM iteration for temporal continuity.
    MatchSet match_frames(const std::vector<KnownPoseFrame>& frames, const ::Params& params,
                          const std::vector<std::vector<int> >& previous) const;

    // Linear pinhole fit from correspondences already in detector order:
    //   u = (fx X + skew Y + cx Z) / Z,  v = (fy Y + cy Z) / Z
    // normalized by Z for conditioning. Distortion is zero in this estimate.
    bool solve_linear_k(const std::vector<Eigen::Vector3d>& points,
                        const std::vector<::Shape>& observed, ::Params* out) const;

    // Per-frame best symmetry + linear K, then one global K from the frames
    // whose per-frame residual passed. Handles per-frame detector relabeling.
    bool linear_bootstrap(const std::vector<KnownPoseFrame>& frames, ::Params* out) const;

    double projection_rms_px(const std::vector<::Shape>& observed, const Eigen::Matrix4d& T,
                             const ::Params& params) const;

    // Ceres with the per-frame poses held at match.T. Fills params and its
    // uncertainty; `reason` is set on failure.
    bool solve_fixed(const std::vector<KnownPoseFrame>& frames, const MatchSet& set,
                     ::Params* params, std::string* reason) const;

    BoardConfig board_;
    KnownPoseOptions options_;
    std::vector<Eigen::Vector3d> lattice_;
    std::vector<LatticeSymmetry> symmetries_;
    std::unique_ptr<BoardProjector> projector_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_KNOWN_POSE_SOLVER_H_
