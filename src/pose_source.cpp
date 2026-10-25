#include "discocal_ros/pose_source.h"

#include <cmath>
#include <cstddef>

namespace discocal_ros {

PoseSource::PoseSource(ros::NodeHandle& nh, const std::string& camera, const std::string& topic,
                       const Handler& handler)
    : camera_(camera), topic_(topic), handler_(handler) {
    if (topic_.empty()) return;
    // The default queue is deliberate: image decoding runs on its own queue,
    // and a pose message must not wait behind a backlog of frames. Pose arrays
    // are small and arrive at the arm's publish rate.
    subscriber_ = nh.subscribe(topic_, 50, &PoseSource::onMessage, this);
    ROS_INFO("discocal_ros: %s: external dot poses on %s (geometry_msgs/PoseArray)",
             camera_.c_str(), topic_.c_str());
}

PoseSource::~PoseSource() {}

void PoseSource::onMessage(const geometry_msgs::PoseArray::ConstPtr& msg) {
    const std::uint64_t stamp = msg->header.stamp.toNSec();
    if (stamp == 0) {
        ROS_WARN_THROTTLE(5.0, "discocal_ros: %s: dropping a pose message on %s with a zero "
                               "header stamp",
                          camera_.c_str(), topic_.c_str());
        return;
    }
    if (msg->poses.empty()) {
        ROS_WARN_THROTTLE(5.0, "discocal_ros: %s: dropping an empty pose message on %s",
                          camera_.c_str(), topic_.c_str());
        return;
    }

    PoseSample sample;
    sample.stamp_ns = static_cast<std::int64_t>(stamp);
    sample.frame_id = msg->header.frame_id;
    sample.points.reserve(msg->poses.size());
    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::Point& p = msg->poses[i].position;
        sample.points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    if (!announced_) {
        announced_ = true;
        ROS_INFO("discocal_ros: %s: first pose message on %s: %zu dot(s), frame \"%s\"",
                 camera_.c_str(), topic_.c_str(), sample.points.size(),
                 sample.frame_id.c_str());
    }

    if (handler_) handler_(sample);
}

}  // namespace discocal_ros
