#ifndef DISCOCAL_ROS_SHA256_H_
#define DISCOCAL_ROS_SHA256_H_

#include <cstddef>
#include <string>

namespace discocal_ros {

// Lower-case hex SHA-256 of `size` bytes at `data`.
//
// Small and dependency-free on purpose: the only consumer is the .dwecal
// sidecar, and pulling OpenSSL into a ROS 1 Melodic package for one hash would
// add a link dependency the vehicle does not otherwise need.
std::string sha256_hex(const void* data, std::size_t size);

std::string sha256_hex(const std::string& data);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_SHA256_H_
