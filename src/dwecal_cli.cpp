// Standalone CLI over the .dwecal generator, for auditing a run's land/
// directory or reproducing the node's file outside ROS:
//
//   dwecal_gen_cli <land dir> <out.dwecal> [src_w src_h dst_w dst_h]
//
// Defaults match dwecal-utils/gen_land.cc: an 800x600 calibration scaled to
// 1600x1200 UXGA.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "discocal_ros/dwecal_gen.h"
#include "discocal_ros/sha256.h"

int main(int argc, char** argv) {
    if (argc != 3 && argc != 7) {
        std::cerr << "Usage: dwecal_gen_cli <land calibration dir> <output dwecal>"
                     " [src_width src_height dst_width dst_height]\n"
                     "  Defaults: 800x600 -> 1600x1200 (UXGA).\n";
        return 2;
    }

    const std::string directory = argv[1];
    const std::string output = argv[2];

    int src_width = 800, src_height = 600;
    int dst_width = 1600, dst_height = 1200;
    if (argc == 7) {
        src_width = std::atoi(argv[3]);
        src_height = std::atoi(argv[4]);
        dst_width = std::atoi(argv[5]);
        dst_height = std::atoi(argv[6]);
    }
    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) {
        std::cerr << "Resolutions must be positive.\n";
        return 2;
    }

    discocal_ros::DwecalInput input;
    std::string reason;
    discocal_ros::DwecalStatus status =
        discocal_ros::load_land_calibration(directory, &input, &reason);
    if (status != discocal_ros::DwecalStatus::kOk) {
        std::cerr << "Failed to load " << directory << ": " << reason << "\n";
        return 1;
    }

    discocal_ros::DwecalResult result;
    status = discocal_ros::generate_dwecal(input, src_width, src_height, dst_width, dst_height,
                                           &result, &reason);
    if (status != discocal_ros::DwecalStatus::kOk) {
        std::cerr << "dwecal generation failed: " << discocal_ros::dwecal_status_message(status);
        if (!reason.empty()) std::cerr << ": " << reason;
        std::cerr << "\n";
        return 1;
    }

    std::ofstream out(output.c_str(), std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "Cannot open " << output << " for writing.\n";
        return 1;
    }
    out.write(reinterpret_cast<const char*>(result.bytes.data()),
              static_cast<std::streamsize>(result.bytes.size()));
    out.flush();
    if (!out.good()) {
        std::cerr << "Failed while writing " << output << ".\n";
        return 1;
    }
    out.close();

    const std::string hash = discocal_ros::sha256_hex(result.bytes.data(), result.bytes.size());
    std::cout << "P0 rectified: fx=" << result.fx << " fy=" << result.fy
              << " cx=" << result.cx << " cy=" << result.cy << "\n";
    std::cout << "Baseline: " << result.baseline_m << " m\n";
    std::cout << "Camera 1 position in camera 0 frame: ["
              << result.camera1_position_in_camera0[0] << ", "
              << result.camera1_position_in_camera0[1] << ", "
              << result.camera1_position_in_camera0[2] << "] m (right: "
              << (result.camera1_is_right ? "yes" : "no") << ")\n";
    std::cout << "Map in-bounds fraction: camera0=" << result.in_bounds_fraction_camera0
              << " camera1=" << result.in_bounds_fraction_camera1 << "\n";
    std::cout << "Wrote " << result.bytes.size() << " bytes, resolution byte "
              << result.resolution_byte << ", sha256 " << hash << "\n";
    return 0;
}
