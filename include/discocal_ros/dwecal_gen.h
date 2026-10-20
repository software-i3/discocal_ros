#ifndef DISCOCAL_ROS_DWECAL_GEN_H_
#define DISCOCAL_ROS_DWECAL_GEN_H_

// .dwecal generation, ported from the standalone dwecal-utils `gen_land.cc`.
//
// The format: a 30-byte header (magic "DWECAL", version 1.1.0, two cameras,
// resolution enum, rectified P0's fx/fy/cx/cy as float32) followed by two
// interleaved float32 (x, y) remap tables, left camera first, row-major. It
// stores no baseline, no checksum and no map of which table belongs to which
// physical camera -- hence DwecalInput::require_camera1_right.
//
// No OpenCV types appear in this header so the generator can be unit-tested and
// linked without pulling OpenCV into consumers that only need the result
// struct. OpenCV is used in the .cpp for stereoRectify only; the maps are built
// by hand because both rigs here have non-zero skew, which OpenCV's own
// initUndistortRectifyMap ignores.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace discocal_ros {

enum class DwecalStatus {
    kOk = 0,
    kParseError,
    kUnsupportedDistortion,
    kUnsupportedResolution,
    kRectifyError,
    kWrongCameraOrder,
    kIoError,
};

const char* dwecal_status_message(DwecalStatus status);

// Resolution enum stored in the header. -1 for a size the format cannot name.
int dwecal_resolution_byte(int width, int height);

struct DwecalInput {
    // Camera intrinsics in the library's field order:
    //   fx, fy, cx, cy, skew, d1, d2, d3, d4
    // d3/d4 must be zero; the generator refuses anything richer rather than
    // silently dropping coefficients.
    std::array<double, 9> camera0 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 9> camera1 = {{0, 0, 0, 0, 0, 0, 0, 0, 0}};

    // The relative pose in stereoRectify's direction:
    //     X_cam1 = R * X_cam0 + T
    // R is row-major, T is metres. This is the TRANSPOSE/NEGATION of the rig
    // config's `p_parent = R p_child + t`; dwecal_bridge does that conversion.
    std::array<double, 9> R = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    std::array<double, 3> T = {{0, 0, 0}};

    // If true, fail with kWrongCameraOrder when camera1's origin in camera0's
    // frame has a non-positive x. With no other record of the map order, a
    // reversed rig would produce mirrored disparity that looks plausible.
    bool require_camera1_right = true;
};

struct DwecalResult {
    std::vector<std::uint8_t> bytes;  // 30 + 2 * width * height * 8

    // Rectified left projection P0, as stored in the header.
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    int width = 0;
    int height = 0;
    int resolution_byte = -1;

    // Diagnostic values, not stored in the file.
    double baseline_m = 0.0;
    std::array<double, 3> camera1_position_in_camera0 = {{0, 0, 0}};
    bool camera1_is_right = false;
    // Fraction of each map's samples that project inside the output image.
    // Alpha = 0 crops, so a healthy rig is near 1.0; a very low value means the
    // overlap or the intrinsics are wrong.
    double in_bounds_fraction_camera0 = 0.0;
    double in_bounds_fraction_camera1 = 0.0;
    double in_bounds_fraction = 0.0;  // min of the two
};

// Generate the file bytes at `dst_width` x `dst_height` from a calibration
// performed at `src_width` x `src_height` (the intrinsics are scaled).
//
// On success returns kOk and fills `result`. On failure `reason` (when non-null)
// carries a human-readable explanation; `result` is left unspecified.
DwecalStatus generate_dwecal(const DwecalInput& input, int src_width, int src_height,
                             int dst_width, int dst_height, DwecalResult* result,
                             std::string* reason);

// Read a land-format calibration directory (camera0_intrinsic.yaml,
// camera1_intrinsic.yaml, relative_camera_poses.txt) into `input`. The pose
// file stores degrees and metres; the conversion to R/T is done here.
//
// This is for the CLI and the golden tests. The node takes its intrinsics and
// extrinsic from the rig config directly and never parses land files.
DwecalStatus load_land_calibration(const std::string& directory, DwecalInput* input,
                                   std::string* reason);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_DWECAL_GEN_H_
