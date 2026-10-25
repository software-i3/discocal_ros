#include "discocal_ros/pose_buffer.h"

#include <algorithm>
#include <cmath>

namespace discocal_ros {

PoseBuffer::PoseBuffer(std::size_t max_samples)
    : max_samples_(max_samples == 0 ? 1 : max_samples) {}

void PoseBuffer::add(const PoseSample& sample) {
    if (sample.stamp_ns <= 0 || sample.points.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Sorted insertion: out-of-order messages are legal on ROS (different
    // publishers, reordering in a relay), and a buffer that only appended
    // would then pair frames with the wrong bracket.
    std::deque<PoseSample>::iterator it = std::lower_bound(
        samples_.begin(), samples_.end(), sample.stamp_ns,
        [](const PoseSample& s, std::int64_t stamp) { return s.stamp_ns < stamp; });
    if (it != samples_.end() && it->stamp_ns == sample.stamp_ns) {
        *it = sample;
    } else {
        samples_.insert(it, sample);
    }

    while (samples_.size() > max_samples_) samples_.pop_front();
}

bool PoseBuffer::lookup(std::int64_t stamp_ns, double tolerance_s, double max_gap_s,
                        bool interpolate, PoseSample* out, double* dt_s,
                        std::int64_t* source_stamp_ns) const {
    if (out == NULL || stamp_ns <= 0 || !(tolerance_s >= 0.0)) return false;

    const double tolerance_ns = tolerance_s * 1e9;
    const double max_gap_ns = max_gap_s * 1e9;

    PoseSample lo;
    PoseSample hi;
    bool have_lo = false;
    bool have_hi = false;
    bool exact = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (samples_.empty()) return false;

        const std::deque<PoseSample>::const_iterator hi_it = std::lower_bound(
            samples_.begin(), samples_.end(), stamp_ns,
            [](const PoseSample& s, std::int64_t stamp) { return s.stamp_ns < stamp; });

        if (hi_it != samples_.end()) {
            hi = *hi_it;
            have_hi = true;
            exact = (hi_it->stamp_ns == stamp_ns);
        }
        if (!exact && hi_it != samples_.begin()) {
            lo = *(hi_it - 1);
            have_lo = true;
        } else if (hi_it == samples_.end() && !samples_.empty()) {
            lo = samples_.back();
            have_lo = true;
            have_hi = false;
        }
    }

    if (exact) {
        *out = hi;
        if (dt_s != NULL) *dt_s = 0.0;
        if (source_stamp_ns != NULL) *source_stamp_ns = hi.stamp_ns;
        return true;
    }

    if (!have_lo && !have_hi) return false;

    if (have_lo && have_hi) {
        const double dt_lo = static_cast<double>(stamp_ns - lo.stamp_ns);
        const double dt_hi = static_cast<double>(hi.stamp_ns - stamp_ns);
        const double gap = static_cast<double>(hi.stamp_ns - lo.stamp_ns);
        const double nearest_dt = std::min(dt_lo, dt_hi);

        const bool can_interpolate = interpolate && nearest_dt <= tolerance_ns &&
                                     gap > 0.0 && gap <= max_gap_ns &&
                                     lo.points.size() == hi.points.size() &&
                                     lo.frame_id == hi.frame_id;
        if (can_interpolate) {
            const double alpha = dt_lo / gap;
            PoseSample blended;
            blended.stamp_ns = stamp_ns;
            blended.frame_id = lo.frame_id;
            blended.points.resize(lo.points.size());
            for (std::size_t i = 0; i < lo.points.size(); ++i) {
                blended.points[i] = (1.0 - alpha) * lo.points[i] + alpha * hi.points[i];
            }
            *out = blended;
            if (dt_s != NULL) *dt_s = nearest_dt / 1e9;
            if (source_stamp_ns != NULL) *source_stamp_ns = lo.stamp_ns;
            return true;
        }

        const PoseSample& nearest = dt_lo <= dt_hi ? lo : hi;
        if (nearest_dt > tolerance_ns) return false;
        *out = nearest;
        if (dt_s != NULL) *dt_s = nearest_dt / 1e9;
        if (source_stamp_ns != NULL) *source_stamp_ns = nearest.stamp_ns;
        return true;
    }

    const PoseSample& only = have_lo ? lo : hi;
    const double distance =
        std::fabs(static_cast<double>(stamp_ns - only.stamp_ns));
    if (distance > tolerance_ns) return false;
    *out = only;
    if (dt_s != NULL) *dt_s = distance / 1e9;
    if (source_stamp_ns != NULL) *source_stamp_ns = only.stamp_ns;
    return true;
}

void PoseBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

std::size_t PoseBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

}  // namespace discocal_ros
