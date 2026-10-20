#ifndef DISCOCAL_ROS_DIVERSITY_GATE_H_
#define DISCOCAL_ROS_DIVERSITY_GATE_H_

// Pose-diversity gate: reject a frame whose board pose is too close to one
// already accepted.
//
// Why it exists. The obvious frame policy -- take the first `budget` frames that
// detect -- reproduces the failure mode measured on this rig: 54 *contiguous*
// frames calibrate 7.30 px away from a 137-frame reference, while 55 *spread*
// frames are only 3.59 px away. Contiguous frames share a pose, the homographies
// they constrain are nearly parallel, fx comes out low and d1 too negative. The
// gate is the streaming analogue of the stride subsets that measured this: it
// keeps the spread without needing the whole sequence up front.
//
// What it compares is a 5-vector signature of the board's appearance in the
// image -- not the pose itself, because the pose is what is being solved for:
//
//     ( cx/W, cy/H, theta/pi, log2(s), aspect )
//
//     cx,cy    centroid of the detected dots
//     W,H      image size, so the first two are resolution-independent
//     theta    in-plane rotation from the dominant eigenvector of the dots'
//              2x2 covariance, wrapped into [0, pi) because the board has
//              180-degree symmetry
//     s        sqrt(dominant eigenvalue)/W -- the board's size in the image
//     aspect   sqrt(lambda2/lambda1), the foreshortening
//
// Distance is the RMS over the five normalised components, so a threshold of
// 0.035 means "every component differs by 3.5% on average". That is what
// frames.diversity_threshold is compared against.

#include <string>
#include <vector>

#include "utils.h"  // ::Shape

namespace discocal_ros {

struct PoseSignature {
    double v[5];
};

// Fills `out` from `shapes` and the image size. False when the signature is not
// defined: fewer than 3 dots, or no spread at all (a degenerate detection).
bool pose_signature(const std::vector<::Shape>& shapes, int width, int height,
                    PoseSignature* out);

// RMS difference of two signatures.
double signature_distance(const PoseSignature& a, const PoseSignature& b);

enum class GateDecision {
    kAccepted,
    kDuplicate,   // too close to something already accepted
    kDegenerate,  // signature undefined; treated as a rejection
};

class DiversityGate {
public:
    DiversityGate(bool enabled, double threshold);

    // Frames are normalised by the image size, so this must be set once before
    // the first `consider`. Until then every call returns kDegenerate.
    void set_image_size(int width, int height);
    bool has_image_size() const { return width_ > 0 && height_ > 0; }

    // Records the frame when accepted. `nearest` reports the distance to the
    // closest accepted signature either way, which is what makes a threshold
    // tunable from the log -- it is the value that was compared.
    GateDecision consider(const std::vector<::Shape>& shapes, double* nearest);

    // Drop every accepted signature. Used when a calibration is retried and the
    // collection starts over, so fresh frames are not all rejected as
    // duplicates of the old attempt's.
    void reset();

    // How many pose signatures are held for comparison. Deliberately 0 while the
    // gate is disabled: a disabled gate accepts every frame *without recording a
    // signature*, since it will never compare one, so there is nothing to count.
    // This is not a count of accepted frames -- the session tracks those itself
    // (SessionStats::accepted), which is the number the feedback and the result
    // report.
    std::size_t accepted() const { return accepted_.size(); }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    bool enabled_;
    double threshold_;
    int width_ = 0;
    int height_ = 0;
    std::vector<PoseSignature> accepted_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_DIVERSITY_GATE_H_
