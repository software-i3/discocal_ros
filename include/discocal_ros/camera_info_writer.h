#ifndef DISCOCAL_ROS_CAMERA_INFO_WRITER_H_
#define DISCOCAL_ROS_CAMERA_INFO_WRITER_H_

// Emits a ROS `camera_info` YAML file -- the format camera_info_manager reads and
// sensor_msgs/CameraInfo maps onto field for field.
//
// The mapping from discocal's parameters is forced by the two models:
//
//     discocal K = [fx  skew  cx]      ROS K = [fx  0(or skew)  cx]
//                  [ 0    fy  cy]              [ 0      fy       cy]
//                  [ 0     0   1]              [ 0       0        1]
//
//     discocal distortion is purely radial, k = 1 + d1 r^2 + d2 r^4 (+ d3 r^6 +
//     d4 r^8). ROS plumb_bob is (k1, k2, p1, p2, k3), so d1 -> k1, d2 -> k2,
//     p1 = p2 = 0 always, and k3 = 0: there is no r^6 term in discocal's model
//     and nothing in the library estimates a tangential one.
//
// Two consequences worth stating plainly:
//
//   * `skew` has no ROS camera_info field. It goes in K[1] and P[1], which is
//     where OpenCV keeps it and what discocal itself means by skew, but a
//     consumer that ignores K[1] is ignoring a real part of the model.
//   * R is identity. A mono calibration has no rectification, and
//     camera_info's R is the rectification rotation only -- not an extrinsic.

#include <cstdint>
#include <string>
#include <vector>

#include "utils.h"  // ::Params

namespace discocal_ros {

struct CameraInfo {
    int width = 0;
    int height = 0;
    std::string camera_name;
    // sensor_msgs/CameraInfo.header.frame_id. Empty falls back to camera_name
    // in make_mono_camera_info(), because a header without a frame is unusable
    // to tf consumers even though camera_info_manager would accept it.
    std::string frame_id;
    std::string distortion_model = "plumb_bob";
    double K[9];
    double D[5];
    double R[9];
    double P[12];

    CameraInfo();
};

// D = {d1, d2, 0, 0, 0}; R = identity; P = K with a zero fourth column.
// `frame_id` is written into the header; empty means "use camera_name".
CameraInfo make_mono_camera_info(const ::Params& params, int width, int height,
                                 const std::string& camera_name,
                                 const std::string& frame_id = "");

std::string camera_info_to_yaml(const CameraInfo& info);

// Creates `path` and any parents. Returns false with `error` set when `path`
// exists as a non-directory or cannot be created. Succeeds if it already exists
// as a directory.
bool ensure_directory(const std::string& path, std::string* error);

// Non-atomic write, kept for callers that do not care about torn files (tests,
// scratch dumps). The calibrators use the atomic variants below.
bool write_text_file(const std::string& path, const std::string& text, std::string* error);

// Writes through <path>.tmp.<pid>.<n>, fsyncs, then renames over `path`. A
// reader sees either the previous file or the complete new one, never a
// half-written calibration -- which matters because the node reports the path
// as soon as the write returns.
bool write_text_file_atomic(const std::string& path, const std::string& text,
                            std::string* error);
bool write_bytes_atomic(const std::string& path, const std::vector<std::uint8_t>& bytes,
                        std::string* error);

// A set of files written together: every temp file is staged and fsynced
// first, and only when all of them succeeded are they renamed into place.
// Used for the two camera_info files, so a failure on the second camera cannot
// leave the first one's file looking like a complete run. (A rename itself can
// still fail; already-renamed files are then left as they are, and the error
// names the failure.)
struct TextFileWrite {
    std::string path;
    std::string text;
};
bool write_text_files_atomic(const std::vector<TextFileWrite>& files, std::string* error);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_CAMERA_INFO_WRITER_H_
