#ifndef DISCOCAL_ROS_FILESYSTEM_COMPAT_H_
#define DISCOCAL_ROS_FILESYSTEM_COMPAT_H_

// discocal_ros: NOT part of upstream discocal. This shim exists only so the
// vendored sources build on the deployment vehicle (Ubuntu 18.04 / ROS Melodic /
// GCC 7.5); upstream targets a newer toolchain and includes <filesystem>
// directly. See THIRD_PARTY_NOTICES.md.
//
// std::filesystem is C++17, but libstdc++ only ships the <filesystem> header
// from GCC 8 on. GCC 7 has the same facilities under their pre-standard name in
// <experimental/filesystem> and no <filesystem> header at all, which fails as
// "fatal error: filesystem: No such file or directory". Include this header
// instead and spell paths discocal_ros::fs::.
//
// GCC 7 and 8 also keep the implementation in a separate libstdc++fs archive;
// CMakeLists.txt links it when the compiler is older than 9.
//
// The alias alone is not the whole port. GCC 7 implements the filesystem *TS*,
// which predates the C++17 additions to directory_entry: query the path with the
// free functions (fs::is_directory(entry.path())) rather than the entry itself
// (entry.is_directory()). The vendored code only ever uses the free functions.

#if defined(__has_include)
#if __has_include(<filesystem>) && __cplusplus >= 201703L
#define DISCOCAL_ROS_HAS_STD_FILESYSTEM 1
#endif
#endif

#ifdef DISCOCAL_ROS_HAS_STD_FILESYSTEM
#include <filesystem>
namespace discocal_ros {
namespace fs = std::filesystem;
}  // namespace discocal_ros
#else
#include <experimental/filesystem>
namespace discocal_ros {
namespace fs = std::experimental::filesystem;
}  // namespace discocal_ros
#endif

#endif  // DISCOCAL_ROS_FILESYSTEM_COMPAT_H_
