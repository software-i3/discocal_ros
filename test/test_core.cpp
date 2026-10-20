// Unit tests for the ROS-free core: config parsing, the lattice, the diversity
// gate, camera_info, and -- the one that matters most -- the cross-camera check.
//
// Nothing here needs ROS, a camera, or the dataset. The check is tested against
// synthetic frames built from a known-good model, so the expected verdict is
// arithmetic rather than a recorded number that would rot.
//
// This file is why CMakeLists guards catkin_add_gtest on the file existing. It
// was referenced before it was written, which broke any build where FindGTest
// succeeded (libgtest-dev ships the headers and, on 18.04, not the archive, so
// whether that happened depended on the machine).
#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include <eigen3/Eigen/Dense>

// The vendored stereo calibrator has `using namespace std;` at global scope.
// Keeping it last in the include list confines the damage to this file.
#include "CStereoCalibrator.h"

#include "discocal_ros/board_frame.h"
#include "discocal_ros/camera_info_writer.h"
#include "discocal_ros/cross_camera_check.h"
#include "discocal_ros/diversity_gate.h"
#include "discocal_ros/dwecal_bridge.h"
#include "discocal_ros/dwecal_gen.h"
#include "discocal_ros/rig_config.h"
#include "discocal_ros/rig_session.h"
#include "discocal_ros/sha256.h"
#include "filesystem_compat.h"
#include "utils.h"  // ::Params, ::Shape

namespace {

using discocal_ros::BoardConfig;
using discocal_ros::BoardFrame;
using discocal_ros::BoardProjector;
using discocal_ros::CameraInfo;
using discocal_ros::CheckResult;
using discocal_ros::CollectionCounters;
using discocal_ros::DiversityGate;
using discocal_ros::DwecalInput;
using discocal_ros::DwecalResult;
using discocal_ros::DwecalStatus;
using discocal_ros::DwecalTiming;
using discocal_ros::ExtrinsicConfig;
using discocal_ros::GateDecision;
using discocal_ros::HeldOutFrame;
using discocal_ros::PerformanceReport;
using discocal_ros::PoseSignature;
using discocal_ros::RigTimings;
using discocal_ros::StageTiming;
using discocal_ros::RigConfig;
using discocal_ros::RigSession;
using discocal_ros::RigSessionResult;
using discocal_ros::board_lattice;
using discocal_ros::camera_info_to_yaml;
using discocal_ros::choose_common_holdout;
using discocal_ros::create_timestamped_run_directory;
using discocal_ros::dwecal_input_from_config;
using discocal_ros::generate_dwecal;
using discocal_ros::load_land_calibration;
using discocal_ros::make_mono_camera_info;
using discocal_ros::run_cross_camera_check;
using discocal_ros::sha256_hex;
using discocal_ros::signature_distance;

// A file in the package tree, resolved from this source file's path. The test
// binary is run from the build tree, so a relative path would depend on cwd.
std::string package_path(const std::string& relative) {
    const std::string source = __FILE__;
    const std::string suffix = "/test/test_core.cpp";
    if (source.size() <= suffix.size() ||
        source.compare(source.size() - suffix.size(), suffix.size(), suffix) != 0) {
        return relative;  // not the layout we expect; let the caller's assertion say so
    }
    return source.substr(0, source.size() - suffix.size()) + "/" + relative;
}

// The example rig's numbers. Kept small and explicit rather than read from the
// config file, so a test failure means behaviour changed, not that the example
// was retuned.
const int kNx = 7;
const int kNy = 5;
const double kRadius = 0.0145;
const double kDistance = 0.039;

::Params make_params(double fx, double fy, double cx, double cy, double skew, double d1,
                     double d2) {
    ::Params p;
    p.K[0] = fx;
    p.K[1] = fy;
    p.K[2] = cx;
    p.K[3] = cy;
    p.K[4] = skew;
    p.d[0] = d1;
    p.d[1] = d2;
    p.d[2] = 0.0;
    p.d[3] = 0.0;
    p.initialize_unc();
    return p;
}

// A DwecalInput with the example rig's measured intrinsics and fixed extrinsic,
// in stereoRectify's direction.
DwecalInput example_dwecal_input() {
    const ::Params left = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                      0.0199394);
    const ::Params right = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    ExtrinsicConfig ext;
    ext.present = true;
    ext.parent = "left";
    ext.child = "right";
    ext.R = discocal_ros::rotation_from_rxryrz(0.005870, 0.013980, -0.013437);
    ext.t = Eigen::Vector3d(0.099966, -0.001510, 0.006461);
    return dwecal_input_from_config(left, right, ext);
}

// A 4x4 board->camera pose from a rotation vector and a translation.
Eigen::Matrix4d pose_of(const Eigen::Vector3d& w, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix();
    T.topRightCorner<3, 1>() = t;
    return T;
}

// The examples' fixed extrinsic (left -> right), as the node reads it.
Eigen::Matrix4d example_extrinsic() {
    return pose_of(Eigen::Vector3d(0.005870, 0.013980, -0.013437),
                   Eigen::Vector3d(0.099966, -0.001510, 0.006461));
}

// ::Shape has no default constructor -- only (cx, cy, area) and the raw-moment
// one -- so build it with the centroid form and leave the moments at zero.
// Only x/y are read by the check.
::Shape shape_at(double x, double y) {
    ::Shape s(x, y, 0.0);
    return s;
}

// Detections for a pose, as ::Shape. Only x/y are read by the check.
std::vector<::Shape> detect(const BoardProjector& proj, const Eigen::Matrix4d& T,
                            const std::vector<Eigen::Vector3d>& lattice,
                            const ::Params& params) {
    std::vector<::Shape> out;
    for (std::size_t k = 0; k < lattice.size(); ++k) {
        const Eigen::Vector2d uv =
            proj.project(lattice[k].x(), lattice[k].y(), kRadius, T, params, 0);
        out.push_back(shape_at(uv.x(), uv.y()));
    }
    return out;
}

// A held-out frame whose two cameras are generated from the SAME pose and the
// SAME extrinsic, so a perfect projector must score ~0. `other_shift_px`
// displaces the other camera's detections to fake a wrong extrinsic or a wrong
// intrinsic set.
HeldOutFrame make_consistent_frame(const std::vector<Eigen::Vector3d>& lattice,
                                   const ::Params& own, const ::Params& other,
                                   const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& T_board,
                                   double other_shift_px, std::int64_t stamp) {
    const BoardProjector proj(2);  // n_d = 2, as BoardConfig defaults to
    HeldOutFrame f;
    f.stamp_ns = stamp;
    f.T = T_board;
    f.own_shapes = detect(proj, T_board, lattice, own);
    f.other_shapes = detect(proj, T_ext * T_board, lattice, other);
    for (std::size_t k = 0; k < f.other_shapes.size(); ++k) {
        f.other_shapes[k].x += other_shift_px;
    }
    return f;
}

// Detections with finite, positive covariance. The joint solver weights each
// residual by the inverse covariance's Cholesky factor, so the zero-covariance
// Shape(double,double,double) used by the cross-check tests cannot be used
// there: its inverse is singular.
std::vector<::Shape> detect_weighted(const BoardProjector& proj, const Eigen::Matrix4d& T,
                                     const std::vector<Eigen::Vector3d>& lattice,
                                     const ::Params& params) {
    std::vector<::Shape> out;
    for (std::size_t k = 0; k < lattice.size(); ++k) {
        const Eigen::Vector2d uv =
            proj.project(lattice[k].x(), lattice[k].y(), kRadius, T, params, 0);
        // n, m00, m10, m01, m20, m11, m02, Kxx, Kxy, Kyy -- an identity
        // information matrix, which leaves the residual unweighted.
        out.push_back(::Shape(1, 1.0, uv.x(), uv.y(), 1.0 / (4.0 * M_PI), 0.0,
                              1.0 / (4.0 * M_PI), 1.0, 0.0, 1.0));
    }
    return out;
}

// A minimal two-camera rig: both calibrate, left is the extrinsic's parent.
RigConfig make_both_rig_config(const std::string& output_directory) {
    RigConfig rig;
    rig.frames.budget = 10;
    rig.frames.diversity_gate = false;

    discocal_ros::CameraConfig left;
    left.name = "left";
    left.topic = "/left/compressed";
    left.transport = "compressed";
    left.frame_id = "explore3d";
    left.source = "calibrate";
    rig.cameras["left"] = left;

    discocal_ros::CameraConfig right = left;
    right.name = "right";
    right.topic = "/right/compressed";
    rig.cameras["right"] = right;

    rig.extrinsic.present = true;
    rig.extrinsic.parent = "left";
    rig.extrinsic.child = "right";
    rig.extrinsic.R = discocal_ros::rotation_from_rxryrz(0.005870, 0.013980, -0.013437);
    rig.extrinsic.t = Eigen::Vector3d(0.099966, -0.001510, 0.006461);

    rig.output.directory = output_directory;
    rig.dwecal.enabled = true;
    rig.dwecal.output_width = 1600;
    rig.dwecal.output_height = 1200;
    return rig;
}

bool contains_stamp(const std::vector<std::int64_t>& stamps, std::int64_t stamp) {
    return std::find(stamps.begin(), stamps.end(), stamp) != stamps.end();
}

// One joint-refinement run, with its own copy of the scene poses because
// StereoCalibrator canonicalises Es in place.
struct JointRun {
    se3 extrinsic;
    std::vector<::Params> params;
    bool converged = false;
};

JointRun run_joint(const std::vector<std::vector<Target> >& targets,
                   const std::vector<se3>& Es, const Eigen::Matrix4d& T_ext,
                   bool fix_extrinsic) {
    JointRun run;
    run.params.push_back(make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394));
    run.params.push_back(make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                     0.0196512));

    std::vector<se3> es = Es;
    std::vector<se3> all_ext(1);
    all_ext[0].rot = LieAlgebra::to_so3(T_ext.topLeftCorner<3, 3>());
    all_ext[0].trans = T_ext.topRightCorner<3, 1>();

    const std::vector<double> radii(2, kRadius);
    const std::vector<double> n_d(2, 2.0);
    StereoCalibrator calibrator(kNx, kNy, false, kDistance);
    calibrator.calibrate(targets, es, all_ext, run.params, radii, n_d, true, fix_extrinsic,
                         &run.converged);
    run.extrinsic = all_ext[0];
    return run;
}

bool identical_se3(const se3& a, const se3& b) {
    for (int i = 0; i < 3; ++i) {
        if (a.rot[i] != b.rot[i] || a.trans[i] != b.trans[i]) return false;
    }
    return true;
}

se3 ext_of(const Eigen::Matrix4d& T) {
    se3 out;
    out.rot = LieAlgebra::to_so3(T.topLeftCorner<3, 3>());
    out.trans = T.topRightCorner<3, 1>();
    return out;
}

// ---------------------------------------------------------------------------

TEST(BoardLattice, OrderMatchesTheDetector) {
    BoardConfig board;
    board.n_x = kNx;
    board.n_y = kNy;
    board.distance = kDistance;
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    ASSERT_EQ(static_cast<std::size_t>(kNx * kNy), lattice.size());

    // Row-major over j (down the board) then i (along the row), with y counted
    // from n_y downwards: the first point is the top-left dot. The cross-camera
    // check compares lattice[k] against observed[k] by index alone, so this
    // ordering is load-bearing -- if it ever changes, every held-out residual
    // becomes a comparison of the wrong pair of dots, which shows up as a large
    // RMS rather than as an error.
    EXPECT_DOUBLE_EQ(kDistance * 1.0, lattice.front().x());
    EXPECT_DOUBLE_EQ(kDistance * kNy, lattice.front().y());
    EXPECT_DOUBLE_EQ(kDistance * kNx, lattice[kNx - 1].x());
    EXPECT_DOUBLE_EQ(kDistance * kNy, lattice[kNx - 1].y());
    for (std::size_t k = 0; k < lattice.size(); ++k) {
        EXPECT_DOUBLE_EQ(0.0, lattice[k].z());
    }
}

TEST(DiversityGate, DisabledAcceptsEverything) {
    DiversityGate gate(false, 0.035);
    gate.set_image_size(800, 600);
    std::vector<::Shape> shapes;
    for (int i = 0; i < kNx * kNy; ++i) {
        shapes.push_back(shape_at(100.0 + i, 200.0));
    }
    double nearest = -1.0;
    EXPECT_EQ(GateDecision::kAccepted, gate.consider(shapes, &nearest));
    EXPECT_EQ(GateDecision::kAccepted, gate.consider(shapes, &nearest));
    EXPECT_DOUBLE_EQ(-1.0, nearest);
    // A disabled gate records no signatures, so accepted() stays 0 -- see the
    // note on the accessor. The frames are accepted all the same.
    EXPECT_EQ(0u, gate.accepted());
}

TEST(DiversityGate, IdenticalPoseIsADuplicate) {
    DiversityGate gate(true, 0.035);
    gate.set_image_size(800, 600);

    std::vector<::Shape> shapes;
    for (int j = 0; j < kNy; ++j) {
        for (int i = 0; i < kNx; ++i) {
            shapes.push_back(shape_at(300.0 + kDistance * i * 1000.0,
                                      250.0 + kDistance * j * 1000.0));
        }
    }
    double nearest = -1.0;
    EXPECT_EQ(GateDecision::kAccepted, gate.consider(shapes, &nearest));

    // Same lattice, moved by a fraction of a pixel: nearer than any threshold
    // this gate would be used with, so it must read as the same pose.
    std::vector<::Shape> same = shapes;
    for (std::size_t k = 0; k < same.size(); ++k) same[k].x += 0.01;
    nearest = -1.0;
    EXPECT_EQ(GateDecision::kDuplicate, gate.consider(same, &nearest));
    EXPECT_GE(nearest, 0.0);

    // A clearly different place in the image is a different pose.
    std::vector<::Shape> moved = shapes;
    for (std::size_t k = 0; k < moved.size(); ++k) moved[k].x += 120.0;
    nearest = -1.0;
    EXPECT_EQ(GateDecision::kAccepted, gate.consider(moved, &nearest));
}

TEST(CameraInfo, MonoLayoutAndPlumbBob) {
    const ::Params p = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                   0.0199394);
    const CameraInfo info = make_mono_camera_info(p, 800, 600, "left");

    EXPECT_EQ(800, info.width);
    EXPECT_EQ(600, info.height);
    EXPECT_STREQ("plumb_bob", info.distortion_model.c_str());
    // No frame_id was passed, so the camera name is the fallback: a header
    // without a frame is not useful to tf consumers.
    EXPECT_STREQ("left", info.frame_id.c_str());

    // OpenCV's K is row-major, so the skew sits at [0][1].
    EXPECT_DOUBLE_EQ(367.451, info.K[0]);
    EXPECT_DOUBLE_EQ(-1.22538, info.K[1]);
    EXPECT_DOUBLE_EQ(398.439, info.K[2]);
    EXPECT_DOUBLE_EQ(368.216, info.K[4]);
    EXPECT_DOUBLE_EQ(324.194, info.K[5]);
    EXPECT_DOUBLE_EQ(1.0, info.K[8]);

    // discocal has no tangential term and n_d = 2, so p1, p2 and k3 are exactly
    // zero rather than merely small.
    EXPECT_DOUBLE_EQ(-0.165002, info.D[0]);
    EXPECT_DOUBLE_EQ(0.0199394, info.D[1]);
    EXPECT_DOUBLE_EQ(0.0, info.D[2]);
    EXPECT_DOUBLE_EQ(0.0, info.D[3]);
    EXPECT_DOUBLE_EQ(0.0, info.D[4]);

    // A mono camera_info is unrectified and has no translation.
    EXPECT_DOUBLE_EQ(1.0, info.R[0]);
    EXPECT_DOUBLE_EQ(1.0, info.R[4]);
    EXPECT_DOUBLE_EQ(1.0, info.R[8]);
    EXPECT_DOUBLE_EQ(0.0, info.R[1]);
    // P is 3x4, so its translation column is at 3, 7 and 11 -- three entries,
    // not four. (Indexing this like the 3x3 arrays reads past the end of P and
    // is how this test first went wrong.)
    for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(0.0, info.P[i * 4 + 3]);
    // The bottom-right of a mono P is the identity row [0, 0, 1, 0].
    EXPECT_DOUBLE_EQ(0.0, info.P[8]);
    EXPECT_DOUBLE_EQ(0.0, info.P[9]);
    EXPECT_DOUBLE_EQ(1.0, info.P[10]);

    const std::string yaml = camera_info_to_yaml(info);
    EXPECT_NE(std::string::npos, yaml.find("image_width: 800"));
    EXPECT_NE(std::string::npos, yaml.find("distortion_model: plumb_bob"));
    EXPECT_NE(std::string::npos, yaml.find("camera_name: left"));
    EXPECT_NE(std::string::npos, yaml.find("rectification_matrix:"));
    EXPECT_NE(std::string::npos, yaml.find("projection_matrix:"));
    // P[0] is fx, and it must be there -- a missing projection_matrix is the
    // failure mode that looks fine to a human reading the file.
    EXPECT_NE(std::string::npos, yaml.find("367.451"));

    // camera_info_manager reads header.frame_id from the same document.
    EXPECT_NE(std::string::npos, yaml.find("header:"));
    EXPECT_NE(std::string::npos, yaml.find("stamp: {secs: 0, nsecs: 0}"));
    EXPECT_NE(std::string::npos, yaml.find("frame_id: left"));

    // An explicit frame (the config's cameras.*.frame_id) wins over the name.
    const CameraInfo framed = make_mono_camera_info(p, 800, 600, "left", "explore3d");
    EXPECT_STREQ("explore3d", framed.frame_id.c_str());
    const std::string framed_yaml = camera_info_to_yaml(framed);
    EXPECT_NE(std::string::npos, framed_yaml.find("frame_id: explore3d"));
    EXPECT_NE(std::string::npos, framed_yaml.find("camera_name: left"));
}

TEST(CameraInfoWriter, AtomicWritesNeverExposePartialFiles) {
    const std::string dir = "/tmp/discocal_ros_test_atomic_" +
                            std::to_string(static_cast<long>(getpid()));
    std::string error;
    ASSERT_TRUE(discocal_ros::ensure_directory(dir, &error)) << error;
    const std::string path = dir + "/left_camera_info.yaml";

    ASSERT_TRUE(discocal_ros::write_text_file_atomic(path, "first\n", &error)) << error;
    {
        std::ifstream in(path.c_str());
        ASSERT_TRUE(in.good());
        const std::string text((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
        EXPECT_EQ("first\n", text);
    }

    // Replacing an existing file is still a rename, so the old contents stay
    // readable up to the instant the new document appears.
    ASSERT_TRUE(discocal_ros::write_text_file_atomic(path, "second\n", &error)) << error;

    // A batch whose second file cannot even be staged must fail before the
    // first file is renamed, leaving the previous contents intact.
    std::vector<discocal_ros::TextFileWrite> batch(2);
    batch[0].path = path;
    batch[0].text = "third\n";
    batch[1].path = dir + "/missing_subdirectory/right_camera_info.yaml";
    batch[1].text = "right\n";
    EXPECT_FALSE(discocal_ros::write_text_files_atomic(batch, &error));
    EXPECT_FALSE(error.empty());
    {
        std::ifstream in(path.c_str());
        ASSERT_TRUE(in.good());
        const std::string text((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
        EXPECT_EQ("second\n", text);
    }

    // No temp file is left behind by either the successful or failed writes.
    for (discocal_ros::fs::directory_iterator it(dir), end; it != end; ++it) {
        const std::string name = it->path().filename().string();
        EXPECT_EQ(std::string::npos, name.find(".tmp.")) << name;
    }

    discocal_ros::fs::remove_all(dir);
}

TEST(PerformanceReport, AccumulatesStagesAndWritesYaml) {
    PerformanceReport report;
    report.begin("both", 100);

    CollectionCounters first;
    first.frames_decoded = 10;
    first.frames_detected = 9;
    first.frames_accepted = 4;
    first.frames_duplicate = 5;
    first.frames_unpaired = 1;
    StageTiming first_time;
    first_time.ran = true;
    first_time.wall_seconds = 2.0;
    first_time.cpu_user_seconds = 1.0;
    first_time.cpu_system_seconds = 0.25;
    report.add_collection(first, first_time);

    // A retry accumulates rather than replacing the first attempt.
    CollectionCounters second;
    second.frames_decoded = 5;
    second.frames_detected = 5;
    second.frames_accepted = 3;
    StageTiming second_time;
    second_time.ran = true;
    second_time.wall_seconds = 1.0;
    second_time.cpu_user_seconds = 0.5;
    second_time.cpu_system_seconds = 0.1;
    report.add_collection(second, second_time);

    RigTimings timings;
    StageTiming mono;
    mono.ran = true;
    mono.wall_seconds = 9.0;
    mono.cpu_user_seconds = 8.0;
    mono.cpu_system_seconds = 0.5;
    timings.mono_solve["left"] = mono;
    timings.mono_solve["right"] = mono;
    timings.mono_fit_seconds = 8.75;
    timings.artifacts = mono;
    report.set_solve_timings(timings);

    DwecalTiming dwecal;
    dwecal.width = 1600;
    dwecal.height = 1200;
    dwecal.bytes = 30720030;
    dwecal.sha256 = "abc";
    dwecal.timing = mono;
    report.set_dwecal(dwecal);
    report.set_attempts(2);
    report.add_warning("a warning");
    report.finish(true, "calibrated both");

    const std::string yaml = report.to_yaml();
    EXPECT_NE(std::string::npos, yaml.find("camera: \"both\""));
    EXPECT_NE(std::string::npos, yaml.find("attempts: 2"));
    EXPECT_NE(std::string::npos, yaml.find("frames_decoded: 15"));
    EXPECT_NE(std::string::npos, yaml.find("frames_accepted: 7"));
    EXPECT_NE(std::string::npos, yaml.find("wall_seconds: 3"));  // collection total
    EXPECT_NE(std::string::npos, yaml.find("left:"));
    EXPECT_NE(std::string::npos, yaml.find("joint_refine:"));
    EXPECT_NE(std::string::npos, yaml.find("bytes: 30720030"));
    EXPECT_NE(std::string::npos, yaml.find("peak_rss_kb:"));
    EXPECT_NE(std::string::npos, yaml.find("ceres_version:"));
    EXPECT_NE(std::string::npos, yaml.find("a warning"));

    const std::string path = "/tmp/discocal_ros_test_performance_" +
                             std::to_string(static_cast<long>(getpid())) + ".yaml";
    std::string error;
    EXPECT_TRUE(report.write(path, &error)) << error;
    std::ifstream in(path.c_str());
    ASSERT_TRUE(in.good());
    const std::string written((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
    EXPECT_EQ(yaml, written);
    discocal_ros::fs::remove(path);
}

TEST(RigConfig, ParsesTheShippedExample) {
    // A minimal file, written inline so the test does not depend on the
    // example's tuning or on where the package is installed.
    const std::string path = "/tmp/discocal_ros_test_rig.yaml";
    {
        std::ofstream out(path.c_str());
        ASSERT_TRUE(out.good());
        out << "board:\n"
            << "  type: circle\n"
            << "  n_x: 7\n"
            << "  n_y: 5\n"
            << "  radius: 0.0145\n"
            << "  distance: 0.039\n"
            << "  edge_check: false\n"
            << "  n_d: 2\n"
            << "detector:\n"
            << "  size_threshold_px2: 120.5\n"
            << "  max_candidates: 40\n"
            << "  edge_contrast: 0.25\n"
            << "  probe: true\n"
            << "performance:\n"
            << "  sparse_info: false\n"
            << "  block_cov: true\n"
            << "  ceres_threads: 3\n"
            << "  opencv_threads: 2\n"
            << "  nice: 5\n"
            << "  async_spinner_threads: 2\n"
            << "solve:\n"
            << "  joint_refine: false\n"
            << "  joint_refine_fallback: false\n"
            << "frames:\n"
            << "  budget: 12\n"
            << "  max_duration_s: 45.5\n"
            << "  diversity_threshold: 0.05\n"
            << "  holdout_fraction: 0.25\n"
            << "check:\n"
            << "  max_rms_px: 7.5\n"
            << "  retries: 0\n"
            << "cameras:\n"
            << "  left:\n"
            << "    topic: /l\n"
            << "    source: calibrate\n"
            << "  right:\n"
            << "    topic: /r\n"
            << "    transport: raw\n"
            << "    source: file\n"
            << "    intrinsics:\n"
            << "      fx: 360.046\n"
            << "      fy: 360.249\n"
            << "      cx: 403.068\n"
            << "      cy: 322.38\n"
            // The distortion is not optional on a `source: file` camera: the
            // parser rejects an intrinsic block without d1/d2, which is how this
            // fixture first failed. A loaded camera that came back with no
            // distortion would silently be a different camera from the one the
            // data was measured with.
            << "      d1: -0.161846\n"
            << "      d2: 0.0196512\n"
            << "      d3: 0.0\n"
            << "      d4: 0.0\n"
            << "extrinsic:\n"
            << "  parent: left\n"
            << "  child: right\n"
            << "  rotation: {rx: 0.005870, ry: 0.013980, rz: -0.013437}\n"
            << "  translation: {x: 0.099966, y: -0.001510, z: 0.006461}\n"
            << "dwecal:\n"
            << "  enabled: true\n"
            << "  output_width: 800\n"
            << "  output_height: 600\n"
            << "  basename: rig.dwecal\n";
        out.close();
    }

    const RigConfig rig = RigConfig::load(path);
    EXPECT_EQ("circle", rig.board.type);
    EXPECT_EQ(kNx, rig.board.n_x);
    EXPECT_DOUBLE_EQ(kDistance, rig.board.distance);
    EXPECT_FALSE(rig.board.edge_check);  // must survive as false, not default true
    EXPECT_EQ(12, rig.frames.budget);
    EXPECT_DOUBLE_EQ(45.5, rig.frames.max_duration_s);
    EXPECT_DOUBLE_EQ(0.05, rig.frames.diversity_threshold);
    EXPECT_DOUBLE_EQ(0.25, rig.frames.holdout_fraction);
    EXPECT_DOUBLE_EQ(7.5, rig.check.max_rms_px);
    EXPECT_EQ(0, rig.check.retries);

    EXPECT_DOUBLE_EQ(120.5, rig.detector.size_threshold_px2);
    EXPECT_EQ(40, rig.detector.max_candidates);
    EXPECT_DOUBLE_EQ(0.25, rig.detector.edge_contrast);
    EXPECT_TRUE(rig.detector.probe);
    EXPECT_FALSE(rig.detector.edge_stats);

    EXPECT_FALSE(rig.performance.sparse_info);  // must survive as false
    EXPECT_TRUE(rig.performance.block_cov);
    EXPECT_EQ(3, rig.performance.ceres_threads);
    EXPECT_EQ(2, rig.performance.opencv_threads);
    EXPECT_EQ(5, rig.performance.nice);
    EXPECT_EQ(2, rig.performance.async_spinner_threads);

    EXPECT_FALSE(rig.solve.joint_refine);
    EXPECT_FALSE(rig.solve.joint_refine_fallback);

    EXPECT_TRUE(rig.dwecal.enabled);
    EXPECT_EQ(800, rig.dwecal.output_width);
    EXPECT_EQ(600, rig.dwecal.output_height);
    EXPECT_EQ("rig.dwecal", rig.dwecal.basename);
    EXPECT_TRUE(rig.dwecal.require_camera1_right);  // default

    ASSERT_TRUE(rig.two_cameras());
    const discocal_ros::CameraConfig* left = rig.camera("left");
    ASSERT_TRUE(left != NULL);
    EXPECT_TRUE(left->source == "calibrate");
    EXPECT_FALSE(left->has_intrinsics);

    const discocal_ros::CameraConfig* right = rig.camera("right");
    ASSERT_TRUE(right != NULL);
    EXPECT_TRUE(right->source == "file");
    EXPECT_TRUE(right->transport == "raw");
    ASSERT_TRUE(right->has_intrinsics);
    EXPECT_DOUBLE_EQ(360.046, right->intrinsics.fx());
    EXPECT_DOUBLE_EQ(322.38, right->intrinsics.cy());

    EXPECT_TRUE(rig.other_camera("left") == "right");
    EXPECT_TRUE(rig.other_camera("right") == "left");

    std::remove(path.c_str());
}

TEST(RigConfig, MissingFileThrows) {
    EXPECT_THROW(RigConfig::load("/tmp/discocal_ros_no_such_rig.yaml"), std::exception);
}

// A minimal valid rig (only `cameras` is required; every other group defaults)
// with `extra` prepended. `extra` must not repeat `cameras`.
std::string write_temp_rig(const char* name, const std::string& extra) {
    const std::string path = std::string("/tmp/") + name + ".yaml";
    std::ofstream out(path.c_str());
    out << extra << "cameras:\n  left:\n    topic: /l\n";
    out.close();
    return path;
}

TEST(RigConfig, RejectsUnrepresentableDwecalResolution) {
    const std::string path = write_temp_rig("discocal_ros_bad_dwecal",
                                            "dwecal:\n  output_width: 1024\n"
                                            "  output_height: 768\n");
    EXPECT_THROW(RigConfig::load(path), std::exception);
    std::remove(path.c_str());
}

TEST(RigConfig, RejectsADwecalPathAsBasename) {
    const std::string path = write_temp_rig("discocal_ros_bad_dwecal_name",
                                            "dwecal:\n  basename: sub/cal.dwecal\n");
    EXPECT_THROW(RigConfig::load(path), std::exception);
    std::remove(path.c_str());
}

TEST(RigConfig, RejectsOutOfRangePerformanceKnobs) {
    const std::string zero_threads =
        write_temp_rig("discocal_ros_bad_ceres", "performance:\n  ceres_threads: 0\n");
    EXPECT_THROW(RigConfig::load(zero_threads), std::exception);
    std::remove(zero_threads.c_str());

    const std::string bad_nice =
        write_temp_rig("discocal_ros_bad_nice", "performance:\n  nice: 20\n");
    EXPECT_THROW(RigConfig::load(bad_nice), std::exception);
    std::remove(bad_nice.c_str());
}

TEST(RigConfig, RejectsANegativeDetectorThreshold) {
    const std::string path = write_temp_rig("discocal_ros_bad_detector",
                                            "detector:\n  size_threshold_px2: -1\n");
    EXPECT_THROW(RigConfig::load(path), std::exception);
    std::remove(path.c_str());
}

TEST(RigConfig, ShippedBothExampleParses) {
    const std::string path = package_path("config/rig_example_both.yaml");
    const RigConfig rig = RigConfig::load(path);
    EXPECT_TRUE(rig.two_cameras());
    ASSERT_TRUE(rig.camera("left") != NULL);
    ASSERT_TRUE(rig.camera("right") != NULL);
    EXPECT_TRUE(rig.camera("left")->source == "calibrate");
    EXPECT_TRUE(rig.camera("right")->source == "calibrate");
    EXPECT_TRUE(rig.extrinsic.present);
    EXPECT_EQ("left", rig.extrinsic.parent);
    EXPECT_EQ("right", rig.extrinsic.child);
    EXPECT_TRUE(rig.solve.joint_refine);
    EXPECT_TRUE(rig.dwecal.enabled);
    EXPECT_EQ(1600, rig.dwecal.output_width);
    EXPECT_EQ(1200, rig.dwecal.output_height);
}

// ---------------------------------------------------------------------------
// SHA-256, the config-to-stereoRectify conversion, and the .dwecal generator.
// ---------------------------------------------------------------------------

TEST(Sha256, KnownVectors) {
    // FIPS 180-4 / the usual published vectors, including the two-block tail
    // and the length encoding.
    EXPECT_EQ("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
              sha256_hex(""));
    EXPECT_EQ("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
              sha256_hex("abc"));
    EXPECT_EQ("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
              sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"));
    EXPECT_EQ("cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
              sha256_hex(std::string(1000000, 'a')));
}

TEST(DwecalBridge, ConversionIsTheInverseOfTheConfigConvention) {
    const ::Params parent = make_params(367.451, 368.216, 398.439, 324.194, -1.22538,
                                        -0.165002, 0.0199394);
    const ::Params child = make_params(360.046, 360.249, 403.068, 322.38, -0.976495,
                                       -0.161846, 0.0196512);
    ExtrinsicConfig ext;
    ext.present = true;
    ext.parent = "left";
    ext.child = "right";
    ext.R = discocal_ros::rotation_from_rxryrz(0.005870, 0.013980, -0.013437);
    ext.t = Eigen::Vector3d(0.099966, -0.001510, 0.006461);

    const DwecalInput in = dwecal_input_from_config(parent, child, ext);

    Eigen::Matrix3d R_land;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) R_land(row, col) = in.R[row * 3 + col];
    }
    const Eigen::Vector3d T_land(in.T[0], in.T[1], in.T[2]);

    // R_config * R_land = I and R_config * T_land + t_config = 0.
    EXPECT_LT((ext.R * R_land - Eigen::Matrix3d::Identity()).norm(), 1e-12);
    EXPECT_LT((ext.R * T_land + ext.t).norm(), 1e-12);

    // The child's origin in the parent's frame comes back as t_config, and for
    // this rig it is on the +x (right) side.
    const Eigen::Vector3d camera1_origin = -R_land.transpose() * T_land;
    EXPECT_LT((camera1_origin - ext.t).norm(), 1e-12);
    EXPECT_GT(camera1_origin.x(), 0.0);
    EXPECT_DOUBLE_EQ(ext.t.norm(), T_land.norm());

    // Field order is fx, fy, cx, cy, skew, d1..d4.
    EXPECT_DOUBLE_EQ(parent.K[0], in.camera0[0]);
    EXPECT_DOUBLE_EQ(parent.K[1], in.camera0[1]);
    EXPECT_DOUBLE_EQ(parent.K[2], in.camera0[2]);
    EXPECT_DOUBLE_EQ(parent.K[3], in.camera0[3]);
    EXPECT_DOUBLE_EQ(parent.K[4], in.camera0[4]);
    EXPECT_DOUBLE_EQ(parent.d[0], in.camera0[5]);
    EXPECT_DOUBLE_EQ(parent.d[1], in.camera0[6]);
    EXPECT_DOUBLE_EQ(child.K[0], in.camera1[0]);
    EXPECT_DOUBLE_EQ(child.d[1], in.camera1[6]);
}

TEST(Dwecal, HeaderAndMapsAtVga) {
    const DwecalInput input = example_dwecal_input();
    DwecalResult result;
    std::string reason;
    ASSERT_EQ(DwecalStatus::kOk,
              generate_dwecal(input, 800, 600, 640, 480, &result, &reason))
        << reason;

    // 30-byte header + two interleaved CV_32FC2 planes.
    EXPECT_EQ(static_cast<std::size_t>(30 + 2 * 640 * 480 * 8), result.bytes.size());
    ASSERT_GE(result.bytes.size(), 30u);
    EXPECT_EQ(0, std::memcmp(result.bytes.data(), "DWECAL", 6));
    EXPECT_EQ(1, result.bytes[6]);  // version major
    EXPECT_EQ(1, result.bytes[7]);  // version minor
    EXPECT_EQ(0, result.bytes[8]);  // version nightly
    EXPECT_EQ(2, result.bytes[9]);  // n_cameras
    EXPECT_EQ(3, result.bytes[10]);  // VGA
    EXPECT_EQ(3, result.resolution_byte);
    EXPECT_EQ(640, result.width);
    EXPECT_EQ(480, result.height);

    float fx = 0, fy = 0, cx = 0, cy = 0;
    std::memcpy(&fx, result.bytes.data() + 14, 4);
    std::memcpy(&fy, result.bytes.data() + 18, 4);
    std::memcpy(&cx, result.bytes.data() + 22, 4);
    std::memcpy(&cy, result.bytes.data() + 26, 4);
    EXPECT_FLOAT_EQ(result.fx, fx);
    EXPECT_FLOAT_EQ(result.fy, fy);
    EXPECT_FLOAT_EQ(result.cx, cx);
    EXPECT_FLOAT_EQ(result.cy, cy);
    EXPECT_GT(result.fx, 0.0f);
    EXPECT_GT(result.fy, 0.0f);

    EXPECT_TRUE(result.camera1_is_right);
    EXPECT_GT(result.camera1_position_in_camera0[0], 0.0);
    EXPECT_NEAR(input.T[0] * input.T[0] + input.T[1] * input.T[1] + input.T[2] * input.T[2],
                result.baseline_m * result.baseline_m, 1e-18);

    // Every map sample must be finite; this is the cheap guard against a bad
    // rectify producing inf/nan that the file would carry silently.
    const std::uint8_t* values = result.bytes.data() + 30;
    for (std::size_t offset = 0; offset + 4 <= result.bytes.size() - 30; offset += 4) {
        float v = 0.0f;
        std::memcpy(&v, values + offset, 4);
        ASSERT_TRUE(std::isfinite(v)) << "map sample " << v << " at byte " << offset;
    }
    EXPECT_GT(result.in_bounds_fraction, 0.5);
}

TEST(Dwecal, RefusesUnrepresentableInputs) {
    const DwecalInput input = example_dwecal_input();
    DwecalResult result;
    std::string reason;

    EXPECT_EQ(DwecalStatus::kUnsupportedResolution,
              generate_dwecal(input, 800, 600, 1024, 768, &result, &reason));
    EXPECT_FALSE(reason.empty());

    DwecalInput bad_distortion = input;
    bad_distortion.camera0[7] = 1e-6;  // d3
    EXPECT_EQ(DwecalStatus::kUnsupportedDistortion,
              generate_dwecal(bad_distortion, 800, 600, 640, 480, &result, &reason));

    // camera1 on the left: T_land = +x with R ~ I puts its origin at -x.
    DwecalInput left_handed = input;
    left_handed.T = {{0.1, 0.0, 0.0}};
    EXPECT_EQ(DwecalStatus::kWrongCameraOrder,
              generate_dwecal(left_handed, 800, 600, 640, 480, &result, &reason));
    left_handed.require_camera1_right = false;
    EXPECT_EQ(DwecalStatus::kOk,
              generate_dwecal(left_handed, 800, 600, 640, 480, &result, &reason));

    DwecalInput bad_focal = input;
    bad_focal.camera1[0] = 0.0;
    EXPECT_EQ(DwecalStatus::kParseError,
              generate_dwecal(bad_focal, 800, 600, 640, 480, &result, &reason));
}

TEST(Dwecal, GoldenLandFixtureMatchesTheRecordedContainerBytes) {
    const std::string directory = package_path("test/data/land_calibration");
    DwecalInput input;
    std::string reason;
    ASSERT_EQ(DwecalStatus::kOk, load_land_calibration(directory, &input, &reason)) << reason;
    EXPECT_DOUBLE_EQ(367.288, input.camera0[0]);
    EXPECT_DOUBLE_EQ(367.408, input.camera1[0]);  // fixture's camera1 fx
    EXPECT_EQ(0.0, input.camera0[7]);

    // These hashes are OpenCV-version-specific: stereoRectify's alpha = 0 ROI
    // search moved between 3.2 and 4.x, and the map is built from its P/H.
    // Recorded in the Melodic container (OpenCV 3.2), which is this package's
    // target. The host-side golden was
    //   UXGA 3be8031117c1fa65280eb751c16988aff1c44dc29015eac6a09f511079854a31
    //   SVGA 51497551e880201845ef2492d31ae49c27c7be4887d7d8c7678c1a335d517afb
    // on OpenCV 4.6.
    DwecalResult uxga;
    ASSERT_EQ(DwecalStatus::kOk,
              generate_dwecal(input, 800, 600, 1600, 1200, &uxga, &reason))
        << reason;
    EXPECT_EQ(30720030u, uxga.bytes.size());
    EXPECT_EQ(0, uxga.resolution_byte);
    // Verified byte-identical to dwecal-utils/gen_land.cc compiled against the
    // same OpenCV 3.2 (Melodic) in this image, on 2026-10-18.
    EXPECT_EQ("62cd49d97bec79a170bced6194b01a0e6e0f5d4d1932b95f8cd81e8916724b5d",
              sha256_hex(uxga.bytes.data(), uxga.bytes.size()));
    EXPECT_NEAR(0.102107, uxga.baseline_m, 1e-6);

    DwecalResult svga;
    ASSERT_EQ(DwecalStatus::kOk,
              generate_dwecal(input, 800, 600, 800, 600, &svga, &reason))
        << reason;
    EXPECT_EQ(7680030u, svga.bytes.size());
    EXPECT_EQ(2, svga.resolution_byte);
    EXPECT_EQ("203a8a80dacf62e3959437386a6bc575ad8423c69d8ff77cb66b58b124062b34",
              sha256_hex(svga.bytes.data(), svga.bytes.size()));
}

// ---------------------------------------------------------------------------
// Joint refinement with a fixed extrinsic.
// ---------------------------------------------------------------------------

TEST(StereoCalibrator, FixedExtrinsicIsBitIdentical) {
    // Detections are generated with the truth transform; the solver is then
    // handed either the truth or a perturbed copy of it. Starting from the
    // truth alone would prove nothing -- a free extrinsic would have no reason
    // to move away from a zero-residual solution.
    const Eigen::Matrix4d T_truth =
        pose_of(Eigen::Vector3d(0.005870, 0.013980, -0.013437),
                Eigen::Vector3d(0.099966, -0.001510, 0.006461));
    Eigen::Matrix4d T_perturbed = T_truth;
    T_perturbed(0, 3) += 0.01;

    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const BoardProjector projector(2);
    const ::Params left = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                      0.0199394);
    const ::Params right = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);

    std::vector<std::vector<Target> > targets(2);
    std::vector<se3> Es;
    for (int i = 0; i < 8; ++i) {
        const double f = static_cast<double>(i);
        const Eigen::Matrix4d T_left_board =
            pose_of(Eigen::Vector3d(0.02 * f - 0.07, 0.012 * f - 0.04, 0.015 * f - 0.05),
                    Eigen::Vector3d(0.01 * f - 0.035, 0.008 * f - 0.028, 0.90 + 0.02 * f));
        const Eigen::Matrix4d T_right_board = T_truth.inverse() * T_left_board;
        targets[0].push_back(Target(true, detect_weighted(projector, T_left_board, lattice, left)));
        targets[1].push_back(
            Target(true, detect_weighted(projector, T_right_board, lattice, right)));
        Es.push_back(LieAlgebra::to_se3(T_left_board));
    }

    // From the truth: the solve should converge and, of course, stay there.
    const JointRun at_truth = run_joint(targets, Es, T_truth, true);
    EXPECT_TRUE(at_truth.converged);
    EXPECT_TRUE(identical_se3(ext_of(T_truth), at_truth.extrinsic));
    for (std::size_t c = 0; c < at_truth.params.size(); ++c) {
        EXPECT_TRUE(std::isfinite(at_truth.params[c].K[0]));
        EXPECT_TRUE(std::isfinite(at_truth.params[c].K[1]));
        EXPECT_GT(at_truth.params[c].K[0], 0.0);
        EXPECT_GT(at_truth.params[c].K[1], 0.0);
    }

    // From a perturbed extrinsic with nonzero residual: a free parameter would
    // move. A fixed one must come back bit for bit, including through
    // normalize_Es(), which is exactly the trap this guards.
    const se3 before = ext_of(T_perturbed);
    const JointRun perturbed = run_joint(targets, Es, T_perturbed, true);
    EXPECT_TRUE(identical_se3(before, perturbed.extrinsic));

    // And the flag must actually mean something: the same run with the
    // extrinsic free is allowed to move it.
    const JointRun free_run = run_joint(targets, Es, T_perturbed, false);
    EXPECT_FALSE(identical_se3(before, free_run.extrinsic));
}

// ---------------------------------------------------------------------------
// The shared holdout and the timestamped run directory.
// ---------------------------------------------------------------------------

TEST(RigSession, CommonHoldoutIsSharedDeterministicAndLeavesTheFitFloor) {
    std::vector<std::int64_t> left;
    std::vector<std::int64_t> right;
    for (int i = 1; i <= 10; ++i) {
        left.push_back(100 * i);
        right.push_back(100 * i);
    }
    right.push_back(1550);  // accepted by one camera only

    const std::vector<std::int64_t> held = choose_common_holdout(left, right, 0.3, 6, 6);
    ASSERT_EQ(3u, held.size());
    // spread_indices(10, 3) -> positions 1, 5, 8.
    EXPECT_EQ(200, held[0]);
    EXPECT_EQ(600, held[1]);
    EXPECT_EQ(900, held[2]);
    EXPECT_FALSE(contains_stamp(held, 1550));

    // The fit sets are accepted-minus-held, so this is the disjointness the
    // check depends on: each camera keeps enough frames and none of the held
    // stamps can be fitted.
    EXPECT_GE(left.size() - held.size(), 6u);
    EXPECT_GE(right.size() - held.size(), 6u);

    // Same stamps in another order, with a duplicate, select the same frames.
    std::vector<std::int64_t> left_reversed(left.rbegin(), left.rend());
    left_reversed.push_back(500);
    EXPECT_EQ(held, choose_common_holdout(left_reversed, right, 0.3, 6, 6));
}

TEST(RigSession, CommonHoldoutIsClampedToTheFitFloor) {
    std::vector<std::int64_t> left;
    std::vector<std::int64_t> right;
    for (int i = 0; i < 8; ++i) {
        left.push_back(i);
        right.push_back(i);
    }
    // Half of 8 would hold out 4, but the left camera must keep 7 -> one only.
    const std::vector<std::int64_t> held = choose_common_holdout(left, right, 0.5, 7, 0);
    ASSERT_EQ(1u, held.size());
    EXPECT_EQ(4, held[0]);
}

TEST(RigSession, CommonHoldoutIsEmptyWhenThereIsNoRoom) {
    std::vector<std::int64_t> a;
    std::vector<std::int64_t> b;
    for (int i = 0; i < 8; ++i) {
        a.push_back(i);
        b.push_back(i);
    }
    EXPECT_TRUE(choose_common_holdout(std::vector<std::int64_t>(), b, 0.2, 0, 0).empty());
    EXPECT_TRUE(choose_common_holdout(a, std::vector<std::int64_t>(), 0.2, 0, 0).empty());
    // No stamp accepted by both cameras.
    std::vector<std::int64_t> disjoint = b;
    for (std::size_t i = 0; i < disjoint.size(); ++i) disjoint[i] += 1000;
    EXPECT_TRUE(choose_common_holdout(a, disjoint, 0.2, 0, 0).empty());
    // The fraction may be zero.
    EXPECT_TRUE(choose_common_holdout(a, b, 0.0, 0, 0).empty());
    // Or the fit floor may leave no room at all.
    EXPECT_TRUE(choose_common_holdout(a, b, 0.5, 8, 8).empty());
}

TEST(RigSession, TimestampedRunDirectoriesNeverCollide) {
    char base_template[] = "/tmp/discocal_ros_run_dirs_XXXXXX";
    char* base = mkdtemp(base_template);
    ASSERT_TRUE(base != NULL) << std::strerror(errno);

    std::string first;
    std::string second;
    std::string error;
    ASSERT_TRUE(create_timestamped_run_directory(base, &first, &error)) << error;
    ASSERT_TRUE(create_timestamped_run_directory(base, &second, &error)) << error;

    const std::string prefix = std::string(base) + "/calibration_";
    EXPECT_NE(first, second);
    EXPECT_EQ(prefix, first.substr(0, prefix.size()));
    EXPECT_EQ(prefix, second.substr(0, prefix.size()));
    EXPECT_EQ(0, access(first.c_str(), F_OK));
    EXPECT_EQ(0, access(second.c_str(), F_OK));

    // Both are empty: the second call did not reuse or clear the first.
    EXPECT_EQ(0, rmdir(first.c_str()));
    EXPECT_EQ(0, rmdir(second.c_str()));
    EXPECT_EQ(0, rmdir(base));
}

TEST(RigSession, TimestampedRunDirectoryRejectsAnUnusableBase) {
    char file_template[] = "/tmp/discocal_ros_run_base_XXXXXX";
    const int fd = mkstemp(file_template);
    ASSERT_GE(fd, 0) << std::strerror(errno);
    ASSERT_EQ(0, close(fd));

    std::string directory;
    std::string error;
    EXPECT_FALSE(create_timestamped_run_directory(file_template, &directory, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(0, unlink(file_template));
}

// ---------------------------------------------------------------------------
// RigSession orchestration.
// ---------------------------------------------------------------------------

TEST(RigSession, RejectsCamerasThatCannotBeSolvedTogether) {
    const RigConfig rig = make_both_rig_config("/tmp/discocal_ros_unused");
    const std::map<std::string, ::Params> known;
    EXPECT_THROW(RigSession(rig, std::vector<std::string>(1, "left"), 10, known),
                 std::runtime_error);
    EXPECT_THROW(RigSession(rig, std::vector<std::string>{"left", "left"}, 10, known),
                 std::runtime_error);
    EXPECT_THROW(RigSession(rig, std::vector<std::string>{"left", "nosuch"}, 10, known),
                 std::runtime_error);

    RigConfig file_rig = rig;
    file_rig.cameras["right"].source = "file";
    EXPECT_THROW(RigSession(file_rig, std::vector<std::string>{"left", "right"}, 10, known),
                 std::runtime_error);
}

TEST(RigSession, NoFramesFailsButStillWritesTheRunSummary) {
    char base_template[] = "/tmp/discocal_ros_rig_run_XXXXXX";
    char* base = mkdtemp(base_template);
    ASSERT_TRUE(base != NULL) << std::strerror(errno);

    const RigConfig rig = make_both_rig_config(base);
    RigSession session(rig, std::vector<std::string>{"left", "right"}, 10,
                       std::map<std::string, ::Params>());
    const RigSessionResult result = session.solve(false);

    EXPECT_FALSE(result.success);
    ASSERT_EQ(2u, result.cameras.size());
    EXPECT_EQ("left", result.cameras[0].camera);
    EXPECT_EQ("right", result.cameras[1].camera);
    EXPECT_FALSE(result.cameras[0].result.success);
    EXPECT_FALSE(result.cameras[1].result.success);
    EXPECT_NE(std::string::npos, result.cameras[0].result.message.find("image size"));
    EXPECT_NE(std::string::npos, result.message.find("image size"));
    EXPECT_FALSE(result.run_directory.empty());
    EXPECT_EQ(result.run_directory + "/land", result.land_directory);
    EXPECT_TRUE(result.dwecal_path.empty());

    const std::string summary_path = result.run_directory + "/rig_calibration.yaml";
    std::ifstream summary(summary_path.c_str());
    ASSERT_TRUE(summary.good()) << summary_path;
    const std::string text((std::istreambuf_iterator<char>(summary)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(std::string::npos, text.find("success: false"));
    EXPECT_NE(std::string::npos, text.find("check_passed: false"));
    EXPECT_NE(std::string::npos, text.find("shared_holdout_frames: 0"));
    EXPECT_NE(std::string::npos, text.find("dwecal not written"));

    EXPECT_EQ(0, std::remove(summary_path.c_str()));
    EXPECT_EQ(0, rmdir(result.land_directory.c_str()));
    EXPECT_EQ(0, rmdir(result.run_directory.c_str()));
    EXPECT_EQ(0, rmdir(base));
}

TEST(RigSession, SyntheticRigCalibratesBothCamerasEndToEnd) {
    // Everything is synthetic, but every stage is real: the diversity gate, two
    // mono solves, the fixed-extrinsic joint refinement, both held-out checks
    // and the artifact writes. The detections are generated from a known model
    // in the config's own convention (p_parent = R p_child + t), so a sign error
    // in the joint extrinsic or in the check's T_own_to_other cannot pass.
    char base_template[] = "/tmp/discocal_ros_rig_e2e_XXXXXX";
    char* base = mkdtemp(base_template);
    ASSERT_TRUE(base != NULL) << std::strerror(errno);

    RigConfig rig = make_both_rig_config(base);
    rig.frames.budget = 10;
    rig.solve.joint_refine = true;
    rig.solve.joint_refine_fallback = true;
    // Keep the artifact small; the 1600x1200 path is covered by the dwecal
    // tests and the real-data run.
    rig.dwecal.output_width = 640;
    rig.dwecal.output_height = 480;

    const ::Params left = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                      0.0199394);
    const ::Params right = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    const Eigen::Matrix4d T_config =
        pose_of(Eigen::Vector3d(0.005870, 0.013980, -0.013437),
                Eigen::Vector3d(0.099966, -0.001510, 0.006461));

    const std::vector<Eigen::Vector3d> lattice = board_lattice(rig.board);
    const BoardProjector projector(rig.board.n_d);

    RigSession session(rig, std::vector<std::string>{"left", "right"}, 10,
                       std::map<std::string, ::Params>());
    session.set_image_size("left", 800, 600);
    session.set_image_size("right", 800, 600);

    for (int i = 0; i < 10; ++i) {
        const double f = static_cast<double>(i);
        const Eigen::Matrix4d T_left_board =
            pose_of(Eigen::Vector3d(0.06 * std::sin(0.7 * f), 0.05 * std::cos(0.9 * f + 0.3),
                                    0.07 * std::sin(0.5 * f + 0.8)),
                    Eigen::Vector3d(-0.16 + 0.03 * std::sin(0.8 * f),
                                    -0.11 + 0.03 * std::cos(0.6 * f), 0.82 + 0.02 * f));
        const Eigen::Matrix4d T_right_board = T_config.inverse() * T_left_board;

        BoardFrame left_frame;
        left_frame.camera = "left";
        left_frame.stamp_ns = 1000 + 100 * i;
        left_frame.shapes = detect_weighted(projector, T_left_board, lattice, left);

        BoardFrame right_frame;
        right_frame.camera = "right";
        right_frame.stamp_ns = left_frame.stamp_ns;
        right_frame.shapes = detect_weighted(projector, T_right_board, lattice, right);

        session.on_detection(left_frame);
        session.on_detection(right_frame);
    }
    ASSERT_TRUE(session.budget_reached());
    EXPECT_EQ(10, session.stats("left").accepted);
    EXPECT_EQ(10, session.stats("right").accepted);

    const RigSessionResult result = session.solve(false);
    ASSERT_TRUE(result.success) << result.message;
    EXPECT_TRUE(result.joint_refined) << result.message;
    EXPECT_GT(result.joint_fit_seconds, 0.0);
    ASSERT_EQ(2u, result.cameras.size());

    const ::Params truth[2] = {left, right};
    for (std::size_t i = 0; i < result.cameras.size(); ++i) {
        const discocal_ros::CameraOutcome& camera = result.cameras[i];
        EXPECT_TRUE(camera.result.success) << camera.result.message;
        EXPECT_TRUE(camera.uncertainties_from_mono_stage);
        EXPECT_TRUE(camera.result.check.ran) << camera.result.check.reason;
        EXPECT_TRUE(camera.result.check.passed) << camera.result.check.reason;
        EXPECT_LT(camera.result.check.rms_px, 0.01);
        EXPECT_EQ(2, camera.result.n_frames_heldout);
        EXPECT_GT(camera.result.fit_seconds, 0.0);
        // The exact-data solve should land on the generating model; the
        // tolerance is loose enough for the library's own coarse/fine stages.
        EXPECT_NEAR(truth[i].K[0], camera.result.params.K[0], 0.05);
        EXPECT_NEAR(truth[i].K[1], camera.result.params.K[1], 0.05);
        EXPECT_NEAR(truth[i].K[2], camera.result.params.K[2], 0.05);
        EXPECT_NEAR(truth[i].K[3], camera.result.params.K[3], 0.05);
    }

    const std::string run = result.run_directory;
    EXPECT_EQ(0, access((run + "/left_camera_info.yaml").c_str(), F_OK));
    EXPECT_EQ(0, access((run + "/right_camera_info.yaml").c_str(), F_OK));
    {
        std::ifstream left_info((run + "/left_camera_info.yaml").c_str());
        ASSERT_TRUE(left_info.good());
        const std::string left_text((std::istreambuf_iterator<char>(left_info)),
                                    std::istreambuf_iterator<char>());
        EXPECT_NE(std::string::npos, left_text.find("header:"));
        // make_both_rig_config sets cameras.*.frame_id: explore3d.
        EXPECT_NE(std::string::npos, left_text.find("frame_id: explore3d"));
    }
    EXPECT_EQ(0, access((run + "/land/camera0_intrinsic.yaml").c_str(), F_OK));
    EXPECT_EQ(0, access((run + "/land/camera1_intrinsic.yaml").c_str(), F_OK));
    EXPECT_EQ(0, access((run + "/land/relative_camera_poses.txt").c_str(), F_OK));
    EXPECT_FALSE(result.dwecal_path.empty());
    EXPECT_EQ(0, access(result.dwecal_path.c_str(), F_OK));
    EXPECT_EQ(0, access((result.dwecal_path + ".sha256").c_str(), F_OK));
    EXPECT_EQ(64u, result.dwecal_sha256.size());

    char header[11] = {0};
    std::ifstream dwecal(result.dwecal_path.c_str(), std::ios::binary);
    ASSERT_TRUE(dwecal.good());
    dwecal.read(header, 11);
    EXPECT_EQ(0, std::memcmp(header, "DWECAL", 6));
    EXPECT_EQ(2, header[9]);  // n_cameras
    EXPECT_EQ(3, header[10]); // VGA (the configured 640x480)

    std::ifstream summary((run + "/rig_calibration.yaml").c_str());
    ASSERT_TRUE(summary.good());
    const std::string text((std::istreambuf_iterator<char>(summary)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(std::string::npos, text.find("check_passed: true"));
    EXPECT_NE(std::string::npos, text.find("joint_refined: true"));
    EXPECT_NE(std::string::npos, text.find("parent_is_camera0: true"));
    EXPECT_NE(std::string::npos, text.find("camera1_is_right: true"));

    discocal_ros::fs::remove_all(run);
    discocal_ros::fs::remove(base);
}

// Feeds `n` synthetic paired detections into a both-camera session configured
// for `budget`, then solves. Every stage is real: two mono solves, the shared
// holdout and both cross-camera checks. Used to pin the collection policy --
// the budget is a ceiling, so a feed that delivers fewer frames is a smaller
// calibration, not a failure.
RigSessionResult solve_synthetic_rig(const char* base, int n, int budget) {
    RigConfig rig = make_both_rig_config(base);
    rig.frames.budget = budget;
    rig.dwecal.output_width = 640;
    rig.dwecal.output_height = 480;

    const ::Params left = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                      0.0199394);
    const ::Params right = make_params(360.046, 360.249, 403.068, 322.38, -0.976495,
                                       -0.161846, 0.0196512);
    const Eigen::Matrix4d T_config =
        pose_of(Eigen::Vector3d(0.005870, 0.013980, -0.013437),
                Eigen::Vector3d(0.099966, -0.001510, 0.006461));
    const std::vector<Eigen::Vector3d> lattice = board_lattice(rig.board);
    const BoardProjector projector(rig.board.n_d);

    RigSession session(rig, std::vector<std::string>{"left", "right"}, budget,
                       std::map<std::string, ::Params>());
    session.set_image_size("left", 800, 600);
    session.set_image_size("right", 800, 600);

    for (int i = 0; i < n; ++i) {
        const double f = static_cast<double>(i);
        const Eigen::Matrix4d T_left_board =
            pose_of(Eigen::Vector3d(0.06 * std::sin(0.7 * f), 0.05 * std::cos(0.9 * f + 0.3),
                                    0.07 * std::sin(0.5 * f + 0.8)),
                    Eigen::Vector3d(-0.16 + 0.03 * std::sin(0.8 * f),
                                    -0.11 + 0.03 * std::cos(0.6 * f), 0.82 + 0.02 * f));
        const Eigen::Matrix4d T_right_board = T_config.inverse() * T_left_board;

        BoardFrame left_frame;
        left_frame.camera = "left";
        left_frame.stamp_ns = 1000 + 100 * i;
        left_frame.shapes = detect_weighted(projector, T_left_board, lattice, left);

        BoardFrame right_frame;
        right_frame.camera = "right";
        right_frame.stamp_ns = left_frame.stamp_ns;
        right_frame.shapes = detect_weighted(projector, T_right_board, lattice, right);

        session.on_detection(left_frame);
        session.on_detection(right_frame);
    }
    return session.solve(false);
}

TEST(RigSession, ShortCollectionAboveTheFloorStillCalibrates) {
    char base_template[] = "/tmp/discocal_ros_short_feed_XXXXXX";
    char* base = mkdtemp(base_template);
    ASSERT_TRUE(base != NULL) << std::strerror(errno);

    // Budget 10, but the feed only ever delivers 8 pairs. That must calibrate:
    // the budget is the most frames to take, not a minimum to reach.
    const RigSessionResult result = solve_synthetic_rig(base, 8, 10);
    EXPECT_TRUE(result.success) << result.message;
    ASSERT_EQ(2u, result.cameras.size());
    for (std::size_t i = 0; i < result.cameras.size(); ++i) {
        EXPECT_TRUE(result.cameras[i].result.success) << result.cameras[i].result.message;
        EXPECT_TRUE(result.cameras[i].result.check.ran) << result.cameras[i].result.message;
        EXPECT_GT(result.cameras[i].result.n_frames, 0u);
        EXPECT_LE(result.cameras[i].result.n_frames + result.cameras[i].result.n_frames_heldout,
                  8u);
    }

    discocal_ros::fs::remove_all(result.run_directory);
    discocal_ros::fs::remove(base);
}

TEST(RigSession, BelowTheSolversFloorFailsWithTheCount) {
    char base_template[] = "/tmp/discocal_ros_starved_feed_XXXXXX";
    char* base = mkdtemp(base_template);
    ASSERT_TRUE(base != NULL) << std::strerror(errno);

    // 4 pairs, against a floor of max(5, 4 + n_d) = 6 for this board: the
    // calibrator cannot initialize, so this is the one short-collection case
    // that must fail -- and the message has to say why rather than just abort.
    const RigSessionResult result = solve_synthetic_rig(base, 4, 10);
    EXPECT_FALSE(result.success);
    EXPECT_NE(std::string::npos, result.message.find("cannot be solved from fewer than 6"))
        << result.message;

    discocal_ros::fs::remove_all(result.run_directory);
    discocal_ros::fs::remove(base);
}

// ---------------------------------------------------------------------------
// The check itself.
// ---------------------------------------------------------------------------

TEST(CrossCameraCheck, ConsistentRigPassesWithNearZeroResidual) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params own = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394);
    const ::Params other = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    const Eigen::Matrix4d T_ext = example_extrinsic();

    std::vector<HeldOutFrame> frames;
    for (int i = 0; i < 5; ++i) {
        // Different board poses, so the residual is not an artefact of one view.
        const Eigen::Matrix4d T_board =
            pose_of(Eigen::Vector3d(0.02 * i, -0.01 * i, 0.05 * i),
                    Eigen::Vector3d(0.01 * i, 0.02 * i, 0.25 + 0.01 * i));
        frames.push_back(make_consistent_frame(lattice, own, other, T_ext, T_board, 0.0, i + 1));
    }

    const CheckResult r = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, other, T_ext,
                                                 1.0);
    EXPECT_TRUE(r.ran);
    EXPECT_EQ(5, r.pairs);
    EXPECT_EQ(5 * kNx * kNy, r.points);
    EXPECT_LT(r.rms_px, 1e-6);  // a perfect model has a perfect residual
    EXPECT_TRUE(r.passed);
    EXPECT_DOUBLE_EQ(1.0, r.limit_px);  // the limit comes back with the verdict
    EXPECT_TRUE(r.reason.empty());
}

TEST(CrossCameraCheck, WrongLimitFlipsTheVerdictAndNotTheMeasurement) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params own = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394);
    const ::Params other = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    const Eigen::Matrix4d T_ext = example_extrinsic();
    const Eigen::Matrix4d T_board =
        pose_of(Eigen::Vector3d(0.03, -0.02, 0.04), Eigen::Vector3d(0.01, 0.01, 0.30));

    // Shift the other camera's detections by 3 px. The residual must be ~3 px,
    // which is the whole point: it is a measurement, not a fit.
    std::vector<HeldOutFrame> frames;
    frames.push_back(make_consistent_frame(lattice, own, other, T_ext, T_board, 3.0, 1));

    const CheckResult loose = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, other,
                                                     T_ext, 10.0);
    ASSERT_TRUE(loose.ran);
    EXPECT_NEAR(3.0, loose.rms_px, 0.05);
    EXPECT_TRUE(loose.passed);

    const CheckResult tight = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, other,
                                                     T_ext, 1.0);
    ASSERT_TRUE(tight.ran);
    EXPECT_FALSE(tight.passed);
    EXPECT_NEAR(loose.rms_px, tight.rms_px, 1e-12);  // same measurement
    EXPECT_FALSE(tight.reason.empty());
    EXPECT_NE(std::string::npos, tight.reason.find("exceeds 1 px"));
}

TEST(CrossCameraCheck, WrongExtrinsicDirectionIsCaught) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params own = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394);
    const ::Params other = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    const Eigen::Matrix4d T_ext = example_extrinsic();
    const Eigen::Matrix4d T_board =
        pose_of(Eigen::Vector3d(0.03, -0.02, 0.04), Eigen::Vector3d(0.01, 0.01, 0.30));

    // Frames generated with the correct extrinsic...
    std::vector<HeldOutFrame> frames;
    for (int i = 0; i < 3; ++i) {
        frames.push_back(make_consistent_frame(lattice, own, other, T_ext, T_board, 0.0, i + 1));
    }

    // ...checked with it read the other way round, which is the mistake the rig
    // config's comment warns about (the libCalib JSON direction).
    const Eigen::Matrix4d wrong = T_ext.inverse();
    const CheckResult r = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, other, wrong,
                                                 1.0);
    EXPECT_TRUE(r.ran);
    EXPECT_FALSE(r.passed);
    EXPECT_GT(r.rms_px, 10.0);  // measured 12.7 px on the real rig for this mistake
}

TEST(CrossCameraCheck, UnpairedAndDegenerateFramesAreCountedNotScored) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params own = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394);
    const ::Params other = make_params(360.046, 360.249, 403.068, 322.38, -0.976495, -0.161846,
                                       0.0196512);
    const Eigen::Matrix4d T_ext = example_extrinsic();
    const Eigen::Matrix4d T_board =
        pose_of(Eigen::Vector3d(0.03, -0.02, 0.04), Eigen::Vector3d(0.01, 0.01, 0.30));

    // One good pair.
    std::vector<HeldOutFrame> frames;
    frames.push_back(make_consistent_frame(lattice, own, other, T_ext, T_board, 0.0, 1));

    // A frame the other camera did not detect: wrong dot count.
    HeldOutFrame unpaired = make_consistent_frame(lattice, own, other, T_ext, T_board, 0.0, 2);
    unpaired.other_shapes.pop_back();
    frames.push_back(unpaired);

    // A frame whose own pose does not reproduce its own detections. The guard
    // must drop it rather than let it judge the extrinsic.
    HeldOutFrame bad_pose = make_consistent_frame(lattice, own, other, T_ext, T_board, 0.0, 3);
    bad_pose.T = pose_of(Eigen::Vector3d(0.5, 0.4, 0.3), Eigen::Vector3d(0.4, 0.4, 0.4));
    frames.push_back(bad_pose);

    const CheckResult r = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, other, T_ext,
                                                 1.0);
    EXPECT_TRUE(r.ran);
    EXPECT_EQ(1, r.pairs);
    EXPECT_EQ(1, r.pairs_unpaired);
    EXPECT_EQ(1, r.pairs_dropped);
    EXPECT_LT(r.rms_px, 1e-6);
}

TEST(CrossCameraCheck, NoUsablePairsDoesNotRun) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params own = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                     0.0199394);
    const Eigen::Matrix4d T_ext = example_extrinsic();

    std::vector<HeldOutFrame> frames;
    HeldOutFrame f;
    f.stamp_ns = 1;
    frames.push_back(f);  // no detections at all

    const CheckResult r = run_cross_camera_check(frames, lattice, kRadius, 2, 0, own, own, T_ext,
                                                 1.0);
    EXPECT_FALSE(r.ran);
    EXPECT_FALSE(r.passed);
    EXPECT_EQ(0, r.pairs);
    EXPECT_FALSE(r.reason.empty());
}

TEST(CrossCameraCheck, EmptyInputReportsWhy) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(BoardConfig());
    const ::Params p = make_params(367.451, 368.216, 398.439, 324.194, -1.22538, -0.165002,
                                   0.0199394);
    const CheckResult r = run_cross_camera_check(std::vector<HeldOutFrame>(), lattice, kRadius, 2,
                                                 0, p, p, example_extrinsic(), 4.0);
    EXPECT_FALSE(r.ran);
    EXPECT_EQ("no held-out frames", r.reason);
}

}  // namespace

// catkin_add_gtest links gtest but not gtest_main, so the suite supplies its own
// entry point. Without this the link fails with "undefined reference to `main'"
// -- which is worth knowing before it happens on the vehicle, where it is the
// difference between the tests running and the package not building.
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
