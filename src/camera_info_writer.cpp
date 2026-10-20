#include "discocal_ros/camera_info_writer.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <unistd.h>

// The GCC 7 filesystem shim, from the vendored tree's include directory (which
// discocal_vendor exports publicly). Needed because <filesystem> does not exist
// on GCC 7 -- see THIRD_PARTY_NOTICES.md.
#include "filesystem_compat.h"

namespace discocal_ros {

CameraInfo::CameraInfo() {
    for (int i = 0; i < 9; ++i) K[i] = 0.0;
    for (int i = 0; i < 5; ++i) D[i] = 0.0;
    for (int i = 0; i < 9; ++i) R[i] = 0.0;
    for (int i = 0; i < 12; ++i) P[i] = 0.0;

    K[0] = 1.0;
    K[4] = 1.0;
    K[8] = 1.0;
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
}

CameraInfo make_mono_camera_info(const ::Params& params, int width, int height,
                                 const std::string& camera_name,
                                 const std::string& frame_id) {
    CameraInfo info;
    info.width = width;
    info.height = height;
    info.camera_name = camera_name;
    // A tf frame has to exist for the header to be useful. The config's
    // cameras.*.frame_id is the authority; the camera name is the fallback so
    // an older config still produces a header a consumer can use.
    info.frame_id = frame_id.empty() ? camera_name : frame_id;

    // Row-major, as camera_info_manager writes it.
    info.K[0] = params.fx();
    info.K[1] = params.skew();
    info.K[2] = params.cx();
    info.K[4] = params.fy();
    info.K[5] = params.cy();
    info.K[8] = 1.0;

    info.D[0] = params.d[0];
    info.D[1] = params.d[1];
    info.D[2] = 0.0;  // p1 -- discocal has no tangential term
    info.D[3] = 0.0;  // p2
    info.D[4] = 0.0;  // k3 -- no r^6 term in the model

    // Unrectified mono: R stays the identity it was constructed as.

    // P is K with a fourth column of zeros, so the stride shifts once per row:
    // P[0..2] = K[0..2], P[4..6] = K[3..5], P[8..10] = K[6..8].
    info.P[0] = info.K[0];
    info.P[1] = info.K[1];
    info.P[2] = info.K[2];
    info.P[3] = 0.0;
    info.P[4] = info.K[3];
    info.P[5] = info.K[4];
    info.P[6] = info.K[5];
    info.P[7] = 0.0;
    info.P[8] = info.K[6];
    info.P[9] = info.K[7];
    info.P[10] = info.K[8];
    info.P[11] = 0.0;

    return info;
}

namespace {

std::string row(int rows, int cols, const double* data, int count) {
    // 17 significant digits round-trips a double exactly, so a consumer that
    // re-reads this file gets the same numbers the calibrator produced rather
    // than a 6-digit shadow of them.
    std::ostringstream oss;
    oss << std::setprecision(17);
    oss << "  rows: " << rows << "\n";
    oss << "  cols: " << cols << "\n";
    oss << "  data: [";
    for (int i = 0; i < count; ++i) {
        if (i > 0) oss << ", ";
        oss << data[i];
    }
    oss << "]\n";
    return oss.str();
}

// Plain YAML scalars stay unquoted (so the files read the way ROS examples
// do); anything with spaces, ':' or a leading special character is quoted.
// camera_info_manager parses with yaml-cpp, which accepts both forms.
std::string yaml_scalar(const std::string& value) {
    bool plain = !value.empty();
    for (std::size_t i = 0; i < value.size() && plain; ++i) {
        const unsigned char c = static_cast<unsigned char>(value[i]);
        plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.' || c == '/';
    }
    if (plain) return value;

    std::string out = "\"";
    for (std::size_t i = 0; i < value.size(); ++i) {
        const char c = value[i];
        if (c == '\\' || c == '"') out += '\\';
        out += c;
    }
    out += '"';
    return out;
}

}  // namespace

std::string camera_info_to_yaml(const CameraInfo& info) {
    std::ostringstream oss;
    oss << "header:\n";
    oss << "  seq: 0\n";
    oss << "  stamp: {secs: 0, nsecs: 0}\n";
    oss << "  frame_id: "
        << yaml_scalar(info.frame_id.empty() ? info.camera_name : info.frame_id) << "\n";
    oss << "image_width: " << info.width << "\n";
    oss << "image_height: " << info.height << "\n";
    oss << "camera_name: " << yaml_scalar(info.camera_name) << "\n";
    oss << "camera_matrix:\n" << row(3, 3, info.K, 9);
    oss << "distortion_model: " << info.distortion_model << "\n";
    oss << "distortion_coefficients:\n" << row(1, 5, info.D, 5);
    oss << "rectification_matrix:\n" << row(3, 3, info.R, 9);
    oss << "projection_matrix:\n" << row(3, 4, info.P, 12);
    return oss.str();
}

bool ensure_directory(const std::string& path, std::string* error) {
    if (path.empty()) {
        if (error != NULL) *error = "path is empty";
        return false;
    }

    const discocal_ros::fs::path p(path);
    std::error_code ec;
    if (discocal_ros::fs::is_directory(p, ec)) return true;
    if (discocal_ros::fs::exists(p, ec)) {
        if (error != NULL) *error = path + " exists and is not a directory";
        return false;
    }

    discocal_ros::fs::create_directories(p, ec);
    if (ec) {
        if (error != NULL) *error = "cannot create " + path + ": " + ec.message();
        return false;
    }
    return true;
}

bool write_text_file(const std::string& path, const std::string& text, std::string* error) {
    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        if (error != NULL) *error = "cannot open " + path + " for writing";
        return false;
    }
    out << text;
    // A camera_info file that was silently dropped on a full disk is worse than
    // no file, because the node would report a path that does not contain the
    // result.
    out.flush();
    if (!out.good()) {
        if (error != NULL) *error = "failed while writing " + path;
        out.close();
        return false;
    }
    out.close();
    return true;
}

namespace {

// Unique per process and per call. The pid keeps two nodes writing the same
// output directory apart; the counter keeps two writes in one process apart.
std::string temp_path_for(const std::string& path) {
    static std::atomic<unsigned long> counter(0);
    std::ostringstream oss;
    oss << path << ".tmp." << static_cast<long>(::getpid()) << "."
        << counter.fetch_add(1);
    return oss.str();
}

bool write_all(int fd, const char* data, std::size_t size, const std::string& path,
               std::string* error) {
    std::size_t written = 0;
    while (written < size) {
        const ssize_t n = ::write(fd, data + written, size - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (error != NULL) {
                *error = "failed while writing " + path + ": " + std::strerror(errno);
            }
            return false;
        }
        written += static_cast<std::size_t>(n);
    }
    return true;
}

// Creates `temp_path` with the full contents, flushes it to disk and closes it.
// On any failure the temp file is removed; `path` itself is never touched.
bool stage_file(const std::string& path, const char* data, std::size_t size,
                std::string* temp_path, std::string* error) {
    const std::string tmp = temp_path_for(path);
    // 0666 & ~umask, matching what an ofstream would create.
    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
    if (fd < 0) {
        if (error != NULL) {
            *error = "cannot open " + tmp + " for writing: " + std::strerror(errno);
        }
        return false;
    }
    if (!write_all(fd, data, size, tmp, error)) {
        ::close(fd);
        ::unlink(tmp.c_str());
        return false;
    }
    // fsync before rename: otherwise a crash can leave the new name pointing at
    // a file whose data has not reached the disk.
    if (::fsync(fd) != 0) {
        if (error != NULL) {
            *error = "cannot flush " + tmp + ": " + std::strerror(errno);
        }
        ::close(fd);
        ::unlink(tmp.c_str());
        return false;
    }
    if (::close(fd) != 0) {
        if (error != NULL) {
            *error = "cannot close " + tmp + ": " + std::strerror(errno);
        }
        ::unlink(tmp.c_str());
        return false;
    }
    *temp_path = tmp;
    return true;
}

bool rename_into_place(const std::string& temp_path, const std::string& path,
                       std::string* error) {
    if (::rename(temp_path.c_str(), path.c_str()) == 0) return true;
    if (error != NULL) {
        *error = "cannot rename " + temp_path + " to " + path + ": " +
                 std::strerror(errno);
    }
    ::unlink(temp_path.c_str());
    return false;
}

}  // namespace

bool write_text_file_atomic(const std::string& path, const std::string& text,
                            std::string* error) {
    std::string tmp;
    if (!stage_file(path, text.data(), text.size(), &tmp, error)) return false;
    return rename_into_place(tmp, path, error);
}

bool write_bytes_atomic(const std::string& path, const std::vector<std::uint8_t>& bytes,
                        std::string* error) {
    const char* data = bytes.empty() ? "" : reinterpret_cast<const char*>(&bytes[0]);
    std::string tmp;
    if (!stage_file(path, data, bytes.size(), &tmp, error)) return false;
    return rename_into_place(tmp, path, error);
}

bool write_text_files_atomic(const std::vector<TextFileWrite>& files, std::string* error) {
    if (files.empty()) return true;

    // Stage every file before renaming any, so a failure while writing camera 1
    // cannot publish camera 0's half of a rig result.
    std::vector<std::string> temps(files.size());
    for (std::size_t i = 0; i < files.size(); ++i) {
        if (!stage_file(files[i].path, files[i].text.data(), files[i].text.size(),
                        &temps[i], error)) {
            for (std::size_t j = 0; j < i; ++j) ::unlink(temps[j].c_str());
            return false;
        }
    }

    for (std::size_t i = 0; i < files.size(); ++i) {
        if (!rename_into_place(temps[i], files[i].path, error)) {
            // Temps already renamed are complete, valid files; leave them.
            for (std::size_t j = i + 1; j < files.size(); ++j) {
                ::unlink(temps[j].c_str());
            }
            return false;
        }
    }
    return true;
}

}  // namespace discocal_ros
