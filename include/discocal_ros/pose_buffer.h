#ifndef DISCOCAL_ROS_POSE_BUFFER_H_
#define DISCOCAL_ROS_POSE_BUFFER_H_

// Time-indexed history of externally measured dot positions, one buffer per
// camera. ROS-free so the interpolation and tolerance policy can be tested
// without a roscore.
//
// The arm moves continuously while a calibration runs, so a pose message is
// only meaningful at the instant it was measured. Each image frame looks up the
// sample at its own header stamp: an exact match when the two feeds share a
// clock, otherwise a linear interpolation between the two bracketing samples.
// A sample farther than the caller's tolerance is not returned at all -- the
// session counts that frame as pose-missing instead of pairing pixels with the
// wrong arm pose.
//
// The buffer is written by the pose subscription callback and read by the
// detection worker, so every method takes the same internal mutex. Reusing one
// buffer for both threads is deliberate: the alternative is a lock in the node
// around every lookup, with the same cost and more places to get it wrong.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

namespace discocal_ros {

struct PoseSample {
    std::int64_t stamp_ns = 0;
    std::string frame_id;
    // Index = the dot's original label. Positions are in the camera's optical
    // frame: z forward, x right, y down.
    std::vector<Eigen::Vector3d> points;
};

class PoseBuffer {
public:
    // ~500 samples is about 16 s at 30 Hz; the node only ever needs the two
    // samples bracketing a live frame.
    explicit PoseBuffer(std::size_t max_samples = 500);

    // Inserts in stamp order, replacing a sample with the same stamp. Samples
    // with a non-positive stamp or no points are ignored: they cannot be
    // paired meaningfully.
    void add(const PoseSample& sample);

    // Finds the sample for `stamp_ns`.
    //
    // With `interpolate` true and two bracketing samples no farther apart than
    // `max_gap_s`, the points are linearly interpolated per label. Otherwise
    // the nearest sample is used. The result is returned only when its distance
    // to `stamp_ns` is within `tolerance_s`; `dt_s` is that distance (for an
    // interpolation, the distance to the nearer bracket).
    //
    // `source_stamp_ns`, when non-null, receives the stamp of the message the
    // points actually came from: the earlier bracket for an interpolation, the
    // sample's own stamp otherwise. It differs from the requested stamp exactly
    // when the result was interpolated or taken from the nearest neighbour.
    //
    // Returns false and leaves `out` untouched when nothing is close enough.
    bool lookup(std::int64_t stamp_ns, double tolerance_s, double max_gap_s, bool interpolate,
                PoseSample* out, double* dt_s, std::int64_t* source_stamp_ns = NULL) const;

    void clear();
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::deque<PoseSample> samples_;
    std::size_t max_samples_;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_POSE_BUFFER_H_
