#ifndef DISOCAL_ROS_IMAGE_SOURCE_H_
#define DISOCAL_ROS_IMAGE_SOURCE_H_

// One camera's image subscription, decoded to the BGR cv::Mat that
// TargetDetector::preprocessing expects.
//
// Decoding happens here and detection happens on the node's single worker
// thread, so this class does no work beyond the conversion. Both the raw
// (sensor_msgs/Image) and compressed (sensor_msgs/CompressedImage) transports
// are always converted to "bgr8": that is what `cv::imread(path,
// IMREAD_COLOR)` gives the offline driver, and preprocessing's channel
// interpretation is written against it.

#include <atomic>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include "discocal_ros/rig_config.h"

namespace discocal_ros {

class ImageSource {
public:
    // `stamp_ns` is the message header stamp in nanoseconds, which is what pairs
    // a frame with the other camera's frame for the same instant.
    typedef void (*Callback)(void* user, const std::string& camera, std::int64_t stamp_ns,
                             const cv::Mat& bgr);

    // Subscribes per `camera.transport`. The callback is invoked from the thread
    // running this subscription's callback queue (the node's image AsyncSpinner),
    // so it must not block -- the node's callback only enqueues.
    //
    // `collecting`, when non-null, is checked at the top of each image callback:
    // while it is false the message is dropped *before* decoding. That is what
    // keeps a solve from spending CPU decoding frames no collection will use.
    ImageSource(ros::NodeHandle& nh, const CameraConfig& camera, Callback callback, void* user,
                const std::atomic<bool>* collecting);

    const std::string& camera() const { return camera_; }

private:
    void onImage(const sensor_msgs::ImageConstPtr& msg);
    void onCompressed(const sensor_msgs::CompressedImageConstPtr& msg);

    // A dropped or undescribable frame is reported once per camera and then
    // counted silently: a permanently incompatible encoding would otherwise
    // fill the log at frame rate.
    void reportProblem(const std::string& what);

    std::string camera_;
    Callback callback_;
    void* user_;
    const std::atomic<bool>* collecting_;
    bool reported_problem_ = false;
    std::size_t dropped_ = 0;

    ros::Subscriber raw_sub_;
    ros::Subscriber compressed_sub_;
};

}  // namespace discocal_ros

#endif  // DISOCAL_ROS_IMAGE_SOURCE_H_
