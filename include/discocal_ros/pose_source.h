#ifndef DISCOCAL_ROS_POSE_SOURCE_H_
#define DISCOCAL_ROS_POSE_SOURCE_H_

// Subscriber for one camera's externally measured target-dot positions.
//
// The contract, which the arm-side publisher implements:
//
//   topic:    CameraConfig::point_pose_topic
//   type:     geometry_msgs/PoseArray
//   header:   stamp = the instant the arm pose (and therefore every dot
//             position) is valid; frame_id = the camera's optical frame
//   poses[i]: position of the dot whose *original* label is i, in that optical
//             frame (z forward, x right, y down). Orientation is ignored.
//
// The array index is a label, not a lattice index. The publisher is free to
// order the dots however the arm/board mounting falls out; the matcher resolves
// the resulting flip/rotation symmetry against the detector's ordering. This
// class therefore does no ordering, thresholding or validation beyond dropping
// messages that cannot be paired at all (zero stamp, no poses). Per-frame
// usability is decided where the frame stamp is known, by the session.

#include <cstdint>
#include <functional>
#include <string>

#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include "discocal_ros/pose_buffer.h"

namespace discocal_ros {

class PoseSource {
public:
    // Called on the ROS callback thread for every usable message.
    typedef std::function<void(const PoseSample&)> Handler;

    PoseSource(ros::NodeHandle& nh, const std::string& camera, const std::string& topic,
               const Handler& handler);
    ~PoseSource();

    const std::string& camera() const { return camera_; }
    const std::string& topic() const { return topic_; }

private:
    PoseSource(const PoseSource&);
    PoseSource& operator=(const PoseSource&);

    void onMessage(const geometry_msgs::PoseArray::ConstPtr& msg);

    std::string camera_;
    std::string topic_;
    Handler handler_;
    ros::Subscriber subscriber_;
    bool announced_ = false;
};

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_POSE_SOURCE_H_
