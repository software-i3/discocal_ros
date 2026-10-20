#include "discocal_ros/image_source.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace discocal_ros {

ImageSource::ImageSource(ros::NodeHandle& nh, const CameraConfig& camera, Callback callback,
                         void* user, const std::atomic<bool>* collecting)
    : camera_(camera.name), callback_(callback), user_(user), collecting_(collecting) {
    // queue_size 1: this callback only enqueues onto the node's own bounded
    // queue, and a backlog here would just be twice-queued frames. The node's
    // queue is where the drop policy lives.
    if (camera.transport == "compressed") {
        compressed_sub_ = nh.subscribe<sensor_msgs::CompressedImage>(
            camera.topic, 1, &ImageSource::onCompressed, this);
    } else {
        raw_sub_ =
            nh.subscribe<sensor_msgs::Image>(camera.topic, 1, &ImageSource::onImage, this);
    }
    ROS_INFO("discocal_ros: %s <- %s (%s)", camera_.c_str(), camera.topic.c_str(),
             camera.transport.c_str());
}

void ImageSource::onImage(const sensor_msgs::ImageConstPtr& msg) {
    if (collecting_ != NULL && !collecting_->load()) return;
    // toCvCopy, not toCvShare: the node keeps the cv::Mat in its queue after
    // this callback returns, and a shared image shares the message's buffer.
    // The queue can outlive the message (ROS may recycle/drop it once every
    // subscriber has returned), so the frame must own its pixels.
    cv_bridge::CvImagePtr bridge;
    try {
        bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        reportProblem(std::string("cannot convert the image to bgr8: ") + e.what());
        return;
    }
    // The stamp can be zero on a driver that does not set it. Pairing by stamp
    // would then pair every frame with every other, so those frames are dropped
    // rather than silently paired at zero.
    if (msg->header.stamp.isZero()) {
        reportProblem("the header stamp is zero, so the frame cannot be paired; dropping");
        return;
    }
    callback_(user_, camera_, static_cast<std::int64_t>(msg->header.stamp.toNSec()), bridge->image);
}

void ImageSource::onCompressed(const sensor_msgs::CompressedImageConstPtr& msg) {
    if (collecting_ != NULL && !collecting_->load()) return;
    cv_bridge::CvImagePtr bridge;
    try {
        bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        reportProblem(std::string("cannot decode the compressed image: ") + e.what());
        return;
    }
    if (msg->header.stamp.isZero()) {
        reportProblem("the header stamp is zero, so the frame cannot be paired; dropping");
        return;
    }
    callback_(user_, camera_, static_cast<std::int64_t>(msg->header.stamp.toNSec()), bridge->image);
}

void ImageSource::reportProblem(const std::string& what) {
    ++dropped_;
    if (reported_problem_) return;
    reported_problem_ = true;
    ROS_WARN("discocal_ros: %s: %s (further occurrences counted silently)", camera_.c_str(),
             what.c_str());
}

}  // namespace discocal_ros
