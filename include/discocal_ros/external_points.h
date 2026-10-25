#ifndef DISCOCAL_ROS_EXTERNAL_POINTS_H_
#define DISCOCAL_ROS_EXTERNAL_POINTS_H_

// Matching a known dot cloud to the detector's dots, and fitting the board pose
// from it.
//
// The external publisher does not know how the board is mounted. It indexes
// each point by the label the dot had before assembly, while the detector
// orders its dots by its own lattice walk. Those two orders coincide only when
// the board is mounted exactly as the nominal lattice assumes; a flip, a 90/180
// degree rotation, or a transpose of a square board permutes one relative to
// the other.
//
// The nominal circle grid has a dihedral symmetry group, and that group is
// exactly the set of relabelings the board can undergo while still looking
// like the same lattice. So the matcher enumerates the lattice's symmetries,
// fits a rigid board pose for each (Kabsch), and scores the candidate by
// projecting the nominal lattice through that pose with the current K/d and the
// library's conic model. The best-scoring symmetry supplies the permutation
// that reorders the external points into detector order.
//
// The rigidity residual of the Kabsch fit is invariant under a relabeling of a
// symmetric lattice, which is why it can validate the point cloud before any
// intrinsics are known. That is the first gate a frame must pass.

#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "discocal_ros/board_frame.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params, ::Shape

namespace discocal_ros {

// One relabeling of the nominal lattice that leaves its point set unchanged.
struct LatticeSymmetry {
    // Index in the order lattice_symmetries() returned.
    int index = 0;
    std::string name;
    // permutation[k] = the lattice index that symmetry k plays. In other
    // words, external[permutation[k]] is the point that belongs with detector
    // dot k.
    std::vector<int> permutation;
};

// Enumerates the lattice's dihedral symmetries about its centroid. The
// identity is always first; the rest are appended in a fixed order
// (rot90, rot180, rot270, flip_x, flip_y, transpose, anti_transpose only when
// the point set actually supports them). Duplicate permutations are removed.
//
// For a 7x5 symmetric grid the survivors are identity, rot180, flip_x and
// flip_y. For a square grid all eight survive. For an asymmetric board the
// result is usually just the identity, which is correct: there is no ambiguity
// to resolve, and the index pairing is whatever the publisher used.
std::vector<LatticeSymmetry> lattice_symmetries(
    const std::vector<Eigen::Vector3d>& lattice);

// Least-squares rigid transform (no scale) taking `model` onto `observed`.
// The model is the nominal lattice; the observed points are in camera frame.
// RMS is the residual after the fit, in the same units as the points.
struct RigidFit {
    bool ok = false;
    std::string reason;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double rms = 0.0;
};
RigidFit fit_rigid_transform(const std::vector<Eigen::Vector3d>& model,
                             const std::vector<Eigen::Vector3d>& observed);

struct BoardMatchOptions {
    // Reject a frame whose point cloud is not a rigid copy of the board.
    double max_rigidity_rms_m = 0.003;
    // Reject the chosen symmetry candidate above this conic reprojection RMS.
    double max_match_rms_px = 3.0;
    // Best and second-best within this are ambiguous. Labels normally make even
    // a fronto-parallel symmetric board solvable -- a wrong pairing puts a
    // dot's measured position on another dot's pixel -- so ambiguity means the
    // pixels cannot separate the pairings: a board so small, distant, or
    // foreshortened that competing symmetries project onto nearly the same
    // points, or noise of that order.
    double min_match_margin_px = 0.25;
    // Keep the previous frame's permutation when it scores within this of the
    // best. 0 disables the preference.
    double temporal_margin_px = 0.5;
    // When false, an ambiguous frame is rejected (ok=false). When true, the
    // best candidate is used and `ambiguous` is still reported.
    bool allow_ambiguous = false;
    // Calibrator projection mode: 0 for circles, 2 for square corners.
    int projection_mode = 0;
};

struct BoardMatch {
    bool ok = false;
    bool ambiguous = false;
    // True when a temporal preference picked a permutation that was not the
    // best-scoring one. Such a frame is not reported ambiguous: continuity is
    // the tie-breaker.
    bool resolved_by_continuity = false;
    std::string reason;

    int symmetry_index = -1;
    std::string symmetry_name;
    // Detector index k -> external label. The reordered point cloud is
    // therefore external[permutation[k]] for detector dot k.
    std::vector<int> permutation;

    // Board frame -> camera frame, fitted from the external points.
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double rigidity_rms_m = 0.0;
    double match_rms_px = 1e300;
    double second_best_rms_px = 1e300;
    double margin_px = 0.0;
};

// Matches one frame.
//
// `lattice` is board_lattice(board) and `external_points` is the PoseArray's
// positions in label order; both must have the same size as `observed`.
// `previous_permutation`, when non-null and of the right size, is the detector
// -> label permutation chosen for an earlier frame; it is preferred when its
// score is within opt.temporal_margin_px of the best.
//
// The caller owns the projector so one MomentsTracker is reused across frames
// and across the solve's EM iterations.
BoardMatch match_board_pose(const std::vector<Eigen::Vector3d>& lattice,
                            const std::vector<Eigen::Vector3d>& external_points,
                            const std::vector<::Shape>& observed, const ::Params& params,
                            double radius, const BoardProjector& projector,
                            const BoardMatchOptions& options,
                            const std::vector<LatticeSymmetry>& symmetries,
                            const std::vector<int>* previous_permutation);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_EXTERNAL_POINTS_H_
