// .dwecal generator. The arithmetic here is a direct port of
// dwecal-utils/gen_land.cc so the node's output is byte-comparable with the
// standalone tool on the same OpenCV build. Comments call out the two places
// that are intentionally not OpenCV's own functions: the skew-aware map and the
// in-bounds audit.

#include "discocal_ros/dwecal_gen.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace discocal_ros {
namespace {

const char* kStatusMessages[] = {
    "ok",
    "could not read or parse the calibration inputs",
    "only two radial distortion terms (d3 = d4 = 0) can be represented",
    "the output resolution is not one of 1600x1200, 1280x720, 800x600, 640x480",
    "OpenCV's stereoRectify failed",
    "camera1 is not to the right of camera0; the maps would be mirrored",
    "the output file could not be written",
};

bool ReadScalars(const std::string& path, std::map<std::string, double>& out) {
    std::ifstream file(path.c_str());
    if (!file.is_open()) return false;
    std::string line;
    while (std::getline(file, line)) {
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        const std::string value = line.substr(colon + 1);
        const std::size_t first = key.find_first_not_of(" \t");
        key.erase(0, first);
        if (!key.empty()) key.erase(key.find_last_not_of(" \t") + 1);
        try {
            out[key] = std::stod(value);
        } catch (...) {
            continue;  // non-numeric value, e.g. img_dir
        }
    }
    return true;
}

bool ReadPose(const std::string& path, double* rvec_deg, double* tvec) {
    std::ifstream file(path.c_str());
    if (!file.is_open()) return false;
    std::string line;
    while (std::getline(file, line)) {
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos || line.compare(0, 1, "T") != 0) continue;
        std::istringstream values(line.substr(colon + 1));
        if (values >> rvec_deg[0] >> rvec_deg[1] >> rvec_deg[2] >> tvec[0] >> tvec[1] >>
            tvec[2]) {
            return true;
        }
    }
    return false;
}

bool ReadCamera(const std::string& path, std::array<double, 9>* camera, std::string* reason) {
    std::map<std::string, double> values;
    if (!ReadScalars(path, values)) {
        *reason = "cannot open " + path;
        return false;
    }
    const char* keys[] = {"fx", "fy", "cx", "cy", "skew", "d1", "d2", "d3", "d4"};
    for (std::size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        if (values.find(keys[i]) == values.end()) {
            *reason = std::string("missing key '") + keys[i] + "' in " + path;
            return false;
        }
    }
    if (values["d3"] != 0.0 || values["d4"] != 0.0) {
        *reason = path + " has non-zero d3/d4; only n_d <= 2 is supported";
        return false;
    }
    (*camera)[0] = values["fx"];
    (*camera)[1] = values["fy"];
    (*camera)[2] = values["cx"];
    (*camera)[3] = values["cy"];
    (*camera)[4] = values["skew"];
    (*camera)[5] = values["d1"];
    (*camera)[6] = values["d2"];
    (*camera)[7] = 0.0;
    (*camera)[8] = 0.0;
    return true;
}

// Rescale the pinhole parameters from the calibration resolution to the
// .dwecal output resolution. Distortion coefficients are dimensionless.
void ScaleCamera(cv::Matx33d* K, double sx, double sy) {
    (*K)(0, 0) *= sx;
    (*K)(0, 1) *= sx;
    (*K)(0, 2) *= sx;
    (*K)(1, 1) *= sy;
    (*K)(1, 2) *= sy;
}

// Destination -> source remap. This deliberately repeats
// cv::initUndistortRectifyMap: OpenCV ignores K(0,1) (skew) in both that
// function and projectPoints, and both cameras here have non-zero skew. With
// skew zero this reproduces OpenCV's map to within float rounding.
void BuildRectifyMap(const cv::Matx33d& K, const cv::Vec<double, 14>& k,
                     const cv::Matx33d& H, const cv::Matx34d& P, const cv::Size& size,
                     cv::Mat* mapX, cv::Mat* mapY, double* in_bounds_fraction) {
    const double fx = K(0, 0), skew = K(0, 1), cx = K(0, 2);
    const double fy = K(1, 1), cy = K(1, 2);
    const double k1 = k[0], k2 = k[1];
    const cv::Matx33d Ht = H.t();

    mapX->create(size, CV_32F);
    mapY->create(size, CV_32F);
    std::size_t in_bounds = 0;
    for (int v = 0; v < size.height; ++v) {
        float* rowX = mapX->ptr<float>(v);
        float* rowY = mapY->ptr<float>(v);
        for (int u = 0; u < size.width; ++u) {
            const double xr = (u - P(0, 2)) / P(0, 0);
            const double yr = (v - P(1, 2)) / P(1, 1);

            const double xx = Ht(0, 0) * xr + Ht(0, 1) * yr + Ht(0, 2);
            const double yy = Ht(1, 0) * xr + Ht(1, 1) * yr + Ht(1, 2);
            const double zz = Ht(2, 0) * xr + Ht(2, 1) * yr + Ht(2, 2);
            const double x = xx / zz;
            const double y = yy / zz;

            const double r2 = x * x + y * y;
            const double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
            const double xd = x * radial;
            const double yd = y * radial;

            const double px = fx * xd + skew * yd + cx;
            const double py = fy * yd + cy;
            rowX[u] = static_cast<float>(px);
            rowY[u] = static_cast<float>(py);
            if (std::isfinite(px) && std::isfinite(py) && px >= 0.0 && px < size.width &&
                py >= 0.0 && py < size.height) {
                ++in_bounds;
            }
        }
    }
    *in_bounds_fraction =
        size.width > 0 && size.height > 0
            ? static_cast<double>(in_bounds) / (static_cast<double>(size.width) * size.height)
            : 0.0;
}

bool AllFinite(const std::array<double, 9>& c) {
    for (std::size_t i = 0; i < c.size(); ++i) {
        if (!std::isfinite(c[i])) return false;
    }
    return true;
}

}  // namespace

const char* dwecal_status_message(DwecalStatus status) {
    const int index = static_cast<int>(status);
    const int count = static_cast<int>(sizeof(kStatusMessages) / sizeof(kStatusMessages[0]));
    return index >= 0 && index < count ? kStatusMessages[index] : "unknown error";
}

int dwecal_resolution_byte(int width, int height) {
    if (width == 1600 && height == 1200) return 0;  // UXGA
    if (width == 1280 && height == 720) return 1;   // HD
    if (width == 800 && height == 600) return 2;    // SVGA
    if (width == 640 && height == 480) return 3;    // VGA
    return -1;
}

DwecalStatus generate_dwecal(const DwecalInput& input, int src_width, int src_height,
                             int dst_width, int dst_height, DwecalResult* result,
                             std::string* reason) {
    if (reason != NULL) reason->clear();
    if (result == NULL) {
        if (reason != NULL) *reason = "result pointer is null";
        return DwecalStatus::kParseError;
    }
    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) {
        if (reason != NULL) *reason = "all resolutions must be positive";
        return DwecalStatus::kUnsupportedResolution;
    }
    const int resolution = dwecal_resolution_byte(dst_width, dst_height);
    if (resolution < 0) {
        std::ostringstream oss;
        oss << "unsupported output resolution " << dst_width << "x" << dst_height;
        if (reason != NULL) *reason = oss.str();
        return DwecalStatus::kUnsupportedResolution;
    }
    if (!AllFinite(input.camera0) || !AllFinite(input.camera1) || !AllFinite(input.R)) {
        if (reason != NULL) *reason = "intrinsics or rotation contain a non-finite value";
        return DwecalStatus::kParseError;
    }
    if (!std::isfinite(input.T[0]) || !std::isfinite(input.T[1]) ||
        !std::isfinite(input.T[2])) {
        if (reason != NULL) *reason = "translation contains a non-finite value";
        return DwecalStatus::kParseError;
    }
    if (input.camera0[0] <= 0.0 || input.camera0[1] <= 0.0 || input.camera1[0] <= 0.0 ||
        input.camera1[1] <= 0.0) {
        if (reason != NULL) *reason = "fx and fy must be positive in both cameras";
        return DwecalStatus::kParseError;
    }
    if (input.camera0[7] != 0.0 || input.camera0[8] != 0.0 || input.camera1[7] != 0.0 ||
        input.camera1[8] != 0.0) {
        if (reason != NULL)
            *reason = "only two radial terms are representable (d3 = d4 = 0 required)";
        return DwecalStatus::kUnsupportedDistortion;
    }

    cv::Matx33d K[2];
    cv::Vec<double, 14> dist[2];
    for (int i = 0; i < 2; ++i) {
        const std::array<double, 9>& c = i == 0 ? input.camera0 : input.camera1;
        K[i] = cv::Matx33d(c[0], c[4], c[2], 0.0, c[1], c[3], 0.0, 0.0, 1.0);
        dist[i] = cv::Vec<double, 14>(c[5], c[6], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0);
    }

    const double sx = static_cast<double>(dst_width) / src_width;
    const double sy = static_cast<double>(dst_height) / src_height;
    ScaleCamera(&K[0], sx, sy);
    ScaleCamera(&K[1], sx, sy);

    // R is supplied as a row-major matrix, not a Rodrigues vector.
    cv::Matx33d R1(input.R[0], input.R[1], input.R[2], input.R[3], input.R[4], input.R[5],
                   input.R[6], input.R[7], input.R[8]);
    const cv::Vec3d tvec(input.T[0], input.T[1], input.T[2]);

    // Baseline / camera order, before rectification. stereoRectify wants
    // X_cam1 = R X_cam0 + T, so camera1's origin in camera0's frame is
    // -R^T T; a positive x means camera1 is the right hand camera.
    const cv::Vec3d cam1_pos = -(R1.t() * tvec);
    const bool camera1_is_right = cam1_pos[0] > 0.0;
    if (input.require_camera1_right && !camera1_is_right) {
        std::ostringstream oss;
        oss << "camera1 origin in camera0 frame is x = " << cam1_pos[0]
            << " m; require_camera1_right is set but camera1 is not to the right";
        if (reason != NULL) *reason = oss.str();
        return DwecalStatus::kWrongCameraOrder;
    }

    const cv::Size frame_size(dst_width, dst_height);
    cv::Matx33d H0, H1;
    cv::Matx34d P0, P1;
    cv::Matx44d Q;
    try {
        cv::stereoRectify(K[0], dist[0], K[1], dist[1], frame_size, R1, tvec, H0, H1, P0, P1,
                          Q, cv::CALIB_ZERO_DISPARITY, 0.0);
    } catch (const cv::Exception& e) {
        if (reason != NULL) *reason = std::string("stereoRectify threw: ") + e.what();
        return DwecalStatus::kRectifyError;
    }

    if (!(P0(0, 0) > 0.0) || !(P0(1, 1) > 0.0) || !std::isfinite(P0(0, 0)) ||
        !std::isfinite(P0(1, 1)) || !std::isfinite(P0(0, 2)) || !std::isfinite(P0(1, 2))) {
        if (reason != NULL) *reason = "stereoRectify produced a non-finite or non-positive P0";
        return DwecalStatus::kRectifyError;
    }

    cv::Mat map0X, map0Y, map1X, map1Y;
    double in_bounds0 = 0.0, in_bounds1 = 0.0;
    BuildRectifyMap(K[0], dist[0], H0, P0, frame_size, &map0X, &map0Y, &in_bounds0);
    BuildRectifyMap(K[1], dist[1], H1, P1, frame_size, &map1X, &map1Y, &in_bounds1);

    // 30-byte header + two interleaved CV_32FC2 maps.
    const std::uint64_t byte_count =
        30ull + 2ull * static_cast<std::uint64_t>(dst_width) * dst_height * 8ull;
    if (byte_count > static_cast<std::uint64_t>(result->bytes.max_size())) {
        if (reason != NULL) *reason = "output is too large for this process";
        return DwecalStatus::kIoError;
    }
    result->bytes.assign(static_cast<std::size_t>(byte_count), 0);

    std::uint8_t* header = result->bytes.data();
    std::memcpy(header, "DWECAL", 6);
    header[6] = 1;  // version major
    header[7] = 1;  // version minor
    header[8] = 0;  // version nightly
    header[9] = 2;  // n_cameras
    header[10] = static_cast<std::uint8_t>(resolution);
    const float fx = P0(0, 0), fy = P0(1, 1), cx = P0(0, 2), cy = P0(1, 2);
    std::memcpy(header + 14, &fx, 4);
    std::memcpy(header + 18, &fy, 4);
    std::memcpy(header + 22, &cx, 4);
    std::memcpy(header + 26, &cy, 4);

    // Interleave, per row: OpenCV rows can carry padding, so pointer access
    // rather than a memcpy over the whole Mat.
    std::uint8_t* out = header + 30;
    const std::size_t plane_bytes = static_cast<std::size_t>(dst_width) * dst_height * 8;
    for (int camera = 0; camera < 2; ++camera) {
        const cv::Mat& mx = camera == 0 ? map0X : map1X;
        const cv::Mat& my = camera == 0 ? map0Y : map1Y;
        for (int y = 0; y < mx.rows; ++y) {
            const float* rowX = mx.ptr<float>(y);
            const float* rowY = my.ptr<float>(y);
            for (int x = 0; x < mx.cols; ++x) {
                // `index` counts floats (two per pixel); `out` is a byte pointer,
                // so the byte offset is index * sizeof(float).
                const std::size_t index =
                    (static_cast<std::size_t>(y) * mx.cols + static_cast<std::size_t>(x)) * 2;
                const std::size_t byte_index = index * sizeof(float);
                std::memcpy(out + byte_index, &rowX[x], 4);
                std::memcpy(out + byte_index + sizeof(float), &rowY[x], 4);
            }
        }
        out += plane_bytes;
    }

    result->fx = fx;
    result->fy = fy;
    result->cx = cx;
    result->cy = cy;
    result->width = dst_width;
    result->height = dst_height;
    result->resolution_byte = resolution;
    result->baseline_m = cv::norm(tvec);
    result->camera1_position_in_camera0 = {{cam1_pos[0], cam1_pos[1], cam1_pos[2]}};
    result->camera1_is_right = camera1_is_right;
    result->in_bounds_fraction_camera0 = in_bounds0;
    result->in_bounds_fraction_camera1 = in_bounds1;
    result->in_bounds_fraction = in_bounds0 < in_bounds1 ? in_bounds0 : in_bounds1;
    return DwecalStatus::kOk;
}

DwecalStatus load_land_calibration(const std::string& directory, DwecalInput* input,
                                   std::string* reason) {
    if (reason != NULL) reason->clear();
    if (input == NULL) {
        if (reason != NULL) *reason = "input pointer is null";
        return DwecalStatus::kParseError;
    }
    std::string dir = directory;
    if (!dir.empty() && dir[dir.size() - 1] != '/') dir += '/';

    std::string detail;
    if (!ReadCamera(dir + "camera0_intrinsic.yaml", &input->camera0, &detail) ||
        !ReadCamera(dir + "camera1_intrinsic.yaml", &input->camera1, &detail)) {
        if (reason != NULL) *reason = detail;
        return DwecalStatus::kParseError;
    }

    double rvec_deg[3] = {0, 0, 0};
    double tvec[3] = {0, 0, 0};
    if (!ReadPose(dir + "relative_camera_poses.txt", rvec_deg, tvec)) {
        if (reason != NULL)
            *reason = "cannot read a T_1^: pose line from " + dir +
                      "relative_camera_poses.txt";
        return DwecalStatus::kParseError;
    }

    // The land file stores the rotation in degrees; the node's config uses
    // radians, and this is the only conversion on the path.
    const cv::Vec3d rvec(rvec_deg[0] * CV_PI / 180.0, rvec_deg[1] * CV_PI / 180.0,
                         rvec_deg[2] * CV_PI / 180.0);
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) input->R[row * 3 + col] = R(row, col);
    }
    input->T = {{tvec[0], tvec[1], tvec[2]}};
    return DwecalStatus::kOk;
}

}  // namespace discocal_ros
