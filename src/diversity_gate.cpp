#include "discocal_ros/diversity_gate.h"

#include <cmath>

namespace discocal_ros {

bool pose_signature(const std::vector<::Shape>& shapes, int width, int height,
                    PoseSignature* out) {
    if (width <= 0 || height <= 0) return false;
    const std::size_t n = shapes.size();
    if (n < 3) return false;

    double mx = 0.0, my = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        mx += shapes[i].x;
        my += shapes[i].y;
    }
    mx /= static_cast<double>(n);
    my /= static_cast<double>(n);

    // Unbiased-ness does not matter here -- this is a shape descriptor, and
    // every frame is scored the same way -- so the plain average is used.
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double dx = shapes[i].x - mx;
        const double dy = shapes[i].y - my;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    cxx /= static_cast<double>(n);
    cxy /= static_cast<double>(n);
    cyy /= static_cast<double>(n);

    const double half = 0.5 * (cxx + cyy);
    const double rad = 0.5 * std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
    const double l1 = half + rad;
    const double l2 = half - rad;
    if (!(l1 > 0.0)) return false;  // every dot at one pixel

    // Dominant eigenvector of the symmetric 2x2 covariance.
    double vx, vy;
    if (std::fabs(cxy) > 0.0) {
        vx = cxy;
        vy = l1 - cxx;
    } else if (cxx >= cyy) {
        vx = 1.0;
        vy = 0.0;
    } else {
        vx = 0.0;
        vy = 1.0;
    }
    const double norm = std::sqrt(vx * vx + vy * vy);
    if (!(norm > 0.0)) return false;
    vx /= norm;
    vy /= norm;

    // The board looks the same rotated by 180 degrees, so the signature must
    // too: wrap the angle into [0, pi).
    double theta = std::atan2(vy, vx);
    while (theta < 0.0) theta += M_PI;
    while (theta >= M_PI) theta -= M_PI;

    const double s = std::sqrt(l1) / static_cast<double>(width);
    if (!(s > 0.0)) return false;
    const double aspect = std::sqrt(std::max(l2, 0.0) / l1);

    out->v[0] = mx / static_cast<double>(width);
    out->v[1] = my / static_cast<double>(height);
    out->v[2] = theta / M_PI;
    // log2, so a factor-of-two change in the board's size is exactly 1.0 and
    // near/far frames are compared as ratios, not as pixel differences.
    out->v[3] = std::log(s) / std::log(2.0);
    out->v[4] = aspect;
    return true;
}

double signature_distance(const PoseSignature& a, const PoseSignature& b) {
    double sum = 0.0;
    for (int i = 0; i < 5; ++i) {
        const double d = a.v[i] - b.v[i];
        sum += d * d;
    }
    return std::sqrt(sum / 5.0);
}

DiversityGate::DiversityGate(bool enabled, double threshold)
    : enabled_(enabled), threshold_(threshold) {}

void DiversityGate::set_image_size(int width, int height) {
    width_ = width;
    height_ = height;
}

void DiversityGate::reset() { accepted_.clear(); }

GateDecision DiversityGate::consider(const std::vector<::Shape>& shapes, double* nearest) {
    if (!enabled_) {
        if (nearest != NULL) *nearest = -1.0;
        return GateDecision::kAccepted;
    }

    PoseSignature sig;
    if (!pose_signature(shapes, width_, height_, &sig)) {
        if (nearest != NULL) *nearest = -1.0;
        return GateDecision::kDegenerate;
    }

    double best = -1.0;
    for (std::size_t i = 0; i < accepted_.size(); ++i) {
        const double d = signature_distance(sig, accepted_[i]);
        if (best < 0.0 || d < best) best = d;
    }
    if (nearest != NULL) *nearest = best;

    // The first frame has nothing to be compared against and is always taken.
    if (best >= 0.0 && best < threshold_) return GateDecision::kDuplicate;

    accepted_.push_back(sig);
    return GateDecision::kAccepted;
}

}  // namespace discocal_ros
