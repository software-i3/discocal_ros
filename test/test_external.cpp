// Tests for the external board-pose path: lattice symmetries, the Kabsch fit,
// symmetry matching, the pose buffer, config parsing, and the known-pose
// intrinsic solve. Synthetic data throughout: the expected answer is arithmetic
// (the pose and K/d used to generate the detections), not a recorded number.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include <eigen3/Eigen/Dense>

#include "discocal_ros/board_frame.h"
#include "discocal_ros/external_points.h"
#include "discocal_ros/known_pose_solver.h"
#include "discocal_ros/pose_buffer.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params, ::Shape

namespace {

using discocal_ros::BoardConfig;
using discocal_ros::BoardMatch;
using discocal_ros::BoardMatchOptions;
using discocal_ros::BoardProjector;
using discocal_ros::ExternalConfig;
using discocal_ros::KnownPoseFrame;
using discocal_ros::KnownPoseOptions;
using discocal_ros::KnownPoseResult;
using discocal_ros::KnownPoseSolver;
using discocal_ros::LatticeSymmetry;
using discocal_ros::PoseBuffer;
using discocal_ros::PoseSample;
using discocal_ros::RigConfig;
using discocal_ros::RigidFit;
using discocal_ros::board_lattice;
using discocal_ros::fit_rigid_transform;
using discocal_ros::lattice_symmetries;
using discocal_ros::match_board_pose;

const double kRadius = 0.0145;

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

Eigen::Matrix4d pose_of(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;
    return T;
}

Eigen::Matrix3d rotation(double ax, double ay, double az) {
    return (Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

::Shape shape_at(double x, double y) { return ::Shape(x, y, 0.0); }

std::vector<::Shape> detect(const BoardProjector& projector,
                            const std::vector<Eigen::Vector3d>& lattice,
                            const Eigen::Matrix4d& T, const ::Params& params) {
    std::vector<::Shape> out;
    for (std::size_t k = 0; k < lattice.size(); ++k) {
        const Eigen::Vector2d uv =
            projector.project(lattice[k].x(), lattice[k].y(), kRadius, T, params, 0);
        out.push_back(shape_at(uv.x(), uv.y()));
    }
    return out;
}

BoardConfig rectangular_board() {
    BoardConfig board;
    board.n_x = 7;
    board.n_y = 5;
    board.radius = kRadius;
    board.distance = 0.039;
    board.n_d = 2;
    return board;
}

BoardConfig square_board() {
    BoardConfig board;
    board.n_x = 4;
    board.n_y = 4;
    board.radius = kRadius;
    board.distance = 0.039;
    board.n_d = 2;
    return board;
}

const LatticeSymmetry* find_symmetry(const std::vector<LatticeSymmetry>& symmetries,
                                     const std::string& name) {
    for (std::size_t i = 0; i < symmetries.size(); ++i) {
        if (symmetries[i].name == name) return &symmetries[i];
    }
    return NULL;
}

std::vector<int> inverse_permutation(const std::vector<int>& permutation) {
    std::vector<int> inverse(permutation.size(), -1);
    for (std::size_t k = 0; k < permutation.size(); ++k) {
        inverse[static_cast<std::size_t>(permutation[k])] = static_cast<int>(k);
    }
    return inverse;
}

bool is_bijection(const std::vector<int>& permutation) {
    std::vector<bool> used(permutation.size(), false);
    for (std::size_t i = 0; i < permutation.size(); ++i) {
        const int value = permutation[i];
        if (value < 0 || static_cast<std::size_t>(value) >= permutation.size() ||
            used[static_cast<std::size_t>(value)]) {
            return false;
        }
        used[static_cast<std::size_t>(value)] = true;
    }
    return true;
}

// The external publisher reports each label at the board-frame position the
// mounting symmetry gives it: label l sits at lattice[s(l)]. The detector's dot
// k looks at lattice[k]. The matcher must therefore pair lattice[k] with
// external[s^-1(k)].
std::vector<Eigen::Vector3d> mounted_points(const std::vector<Eigen::Vector3d>& lattice,
                                            const std::vector<int>& s, const Eigen::Matrix4d& T) {
    std::vector<Eigen::Vector3d> points(lattice.size());
    for (std::size_t label = 0; label < lattice.size(); ++label) {
        const Eigen::Vector3d local = lattice[static_cast<std::size_t>(s[label])];
        points[label] = T.topLeftCorner<3, 3>() * local + T.topRightCorner<3, 1>();
    }
    return points;
}

std::string temp_path(const std::string& suffix) {
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), "/tmp/discocal_external_%d_%s", getpid(),
                  suffix.c_str());
    return std::string(buffer);
}

// ---------------------------------------------------------------------------
// Symmetry enumeration
// ---------------------------------------------------------------------------

TEST(ExternalPoints, RectangularGridHasFourSymmetries) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(rectangular_board());
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);

    ASSERT_EQ(4u, symmetries.size());
    EXPECT_EQ("identity", symmetries[0].name);
    EXPECT_TRUE(find_symmetry(symmetries, "rot180") != NULL);
    EXPECT_TRUE(find_symmetry(symmetries, "flip_x") != NULL);
    EXPECT_TRUE(find_symmetry(symmetries, "flip_y") != NULL);
    EXPECT_TRUE(find_symmetry(symmetries, "rot90") == NULL);
    for (std::size_t i = 0; i < symmetries.size(); ++i) {
        EXPECT_TRUE(is_bijection(symmetries[i].permutation)) << symmetries[i].name;
        EXPECT_EQ(lattice.size(), symmetries[i].permutation.size());
    }
}

TEST(ExternalPoints, SquareGridHasAllEightSymmetries) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(square_board());
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);

    ASSERT_EQ(8u, symmetries.size());
    for (std::size_t i = 0; i < symmetries.size(); ++i) {
        EXPECT_TRUE(is_bijection(symmetries[i].permutation)) << symmetries[i].name;
    }
}

// ---------------------------------------------------------------------------
// Kabsch
// ---------------------------------------------------------------------------

TEST(ExternalPoints, RigidFitRecoversThePose) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(rectangular_board());
    const Eigen::Matrix4d T =
        pose_of(rotation(0.2, -0.35, 0.1), Eigen::Vector3d(0.03, -0.02, 0.8));

    std::vector<Eigen::Vector3d> observed(lattice.size());
    for (std::size_t i = 0; i < lattice.size(); ++i) {
        observed[i] = T.topLeftCorner<3, 3>() * lattice[i] + T.topRightCorner<3, 1>();
    }

    const RigidFit fit = fit_rigid_transform(lattice, observed);
    ASSERT_TRUE(fit.ok);
    EXPECT_LT(fit.rms, 1e-12);
    EXPECT_LT((fit.T - T).norm(), 1e-9);
}

TEST(ExternalPoints, RigidFitRejectsCountMismatch) {
    const std::vector<Eigen::Vector3d> lattice = board_lattice(rectangular_board());
    std::vector<Eigen::Vector3d> observed(lattice.size() - 1, Eigen::Vector3d::Zero());
    const RigidFit fit = fit_rigid_transform(lattice, observed);
    EXPECT_FALSE(fit.ok);
    EXPECT_FALSE(fit.reason.empty());
}

// ---------------------------------------------------------------------------
// Symmetry matching
// ---------------------------------------------------------------------------

TEST(ExternalPoints, MatchResolvesFlippedRectangularBoard) {
    const BoardConfig board = rectangular_board();
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);
    const LatticeSymmetry* flip = find_symmetry(symmetries, "flip_x");
    ASSERT_TRUE(flip != NULL);

    const ::Params params = make_params(800.0, 810.0, 320.0, 240.0, 0.05, -0.04, 0.01);
    const Eigen::Matrix4d T =
        pose_of(rotation(0.35, -0.2, 0.15), Eigen::Vector3d(0.01, -0.03, 0.65));
    const BoardProjector projector(board.n_d);
    const std::vector<::Shape> shapes = detect(projector, lattice, T, params);

    const std::vector<Eigen::Vector3d> external =
        mounted_points(lattice, flip->permutation, T);
    const std::vector<int> expected = inverse_permutation(flip->permutation);

    BoardMatchOptions options;
    const BoardMatch match = match_board_pose(lattice, external, shapes, params, kRadius,
                                              projector, options, symmetries, NULL);

    ASSERT_TRUE(match.ok) << match.reason;
    EXPECT_FALSE(match.ambiguous);
    EXPECT_LT(match.match_rms_px, 1e-6);
    EXPECT_LT(match.rigidity_rms_m, 1e-9);
    ASSERT_EQ(expected.size(), match.permutation.size());
    EXPECT_EQ(expected, match.permutation);
    EXPECT_LT((match.T - T).norm(), 1e-9);
}

TEST(ExternalPoints, MatchResolvesRotatedSquareBoard) {
    const BoardConfig board = square_board();
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);
    const LatticeSymmetry* rot = find_symmetry(symmetries, "rot90");
    ASSERT_TRUE(rot != NULL);

    const ::Params params = make_params(700.0, 705.0, 318.0, 242.0, 0.0, -0.03, 0.005);
    const Eigen::Matrix4d T =
        pose_of(rotation(-0.25, 0.4, -0.3), Eigen::Vector3d(-0.02, 0.015, 0.55));
    const BoardProjector projector(board.n_d);
    const std::vector<::Shape> shapes = detect(projector, lattice, T, params);

    const std::vector<Eigen::Vector3d> external =
        mounted_points(lattice, rot->permutation, T);

    BoardMatchOptions options;
    const BoardMatch match = match_board_pose(lattice, external, shapes, params, kRadius,
                                              projector, options, symmetries, NULL);

    ASSERT_TRUE(match.ok) << match.reason;
    EXPECT_LT(match.match_rms_px, 1e-6);
    EXPECT_EQ(inverse_permutation(rot->permutation), match.permutation);
    EXPECT_LT((match.T - T).norm(), 1e-9);
}

TEST(ExternalPoints, MatchRejectsNonRigidCloud) {
    const BoardConfig board = rectangular_board();
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);

    const ::Params params = make_params(800.0, 800.0, 320.0, 240.0, 0.0, 0.0, 0.0);
    const Eigen::Matrix4d T = pose_of(rotation(0.3, -0.2, 0.0), Eigen::Vector3d(0.0, 0.0, 0.6));
    const BoardProjector projector(board.n_d);
    const std::vector<::Shape> shapes = detect(projector, lattice, T, params);

    std::vector<int> identity(lattice.size());
    for (std::size_t i = 0; i < lattice.size(); ++i) identity[i] = static_cast<int>(i);
    std::vector<Eigen::Vector3d> external = mounted_points(lattice, identity, T);
    external[0].x() += 0.02;  // 20 mm: not a rigid copy of the board

    BoardMatchOptions options;
    const BoardMatch match = match_board_pose(lattice, external, shapes, params, kRadius,
                                              projector, options, symmetries, NULL);
    EXPECT_FALSE(match.ok);
    EXPECT_NE(std::string::npos, match.reason.find("rigid"));
}

TEST(ExternalPoints, FrontoParallelSquareBoardIsResolvedByLabels) {
    const BoardConfig board = square_board();
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);

    // A face-on square board cannot be oriented from the image alone, but the
    // external labels are attached to physical dots, and the detector's index
    // ordering is fixed, so exactly one symmetry explains both. This is the
    // reason the external path does not inherit the image path's pose ambiguity.
    const ::Params params = make_params(800.0, 800.0, 320.0, 240.0, 0.0, 0.0, 0.0);
    const Eigen::Matrix4d T = pose_of(Eigen::Matrix3d::Identity(),
                                      Eigen::Vector3d(0.0, 0.0, 0.6));
    const BoardProjector projector(board.n_d);
    const std::vector<::Shape> shapes = detect(projector, lattice, T, params);

    std::vector<int> identity(lattice.size());
    for (std::size_t i = 0; i < lattice.size(); ++i) identity[i] = static_cast<int>(i);
    const std::vector<Eigen::Vector3d> external = mounted_points(lattice, identity, T);

    BoardMatchOptions options;
    const BoardMatch match = match_board_pose(lattice, external, shapes, params, kRadius,
                                              projector, options, symmetries, NULL);
    ASSERT_TRUE(match.ok) << match.reason;
    EXPECT_FALSE(match.ambiguous);
    EXPECT_EQ("identity", match.symmetry_name);
    EXPECT_EQ(identity, match.permutation);
    EXPECT_LT(match.match_rms_px, 1e-6);
}

TEST(ExternalPoints, TwoByTwoBoardMatchesWithLabels) {
    // External mode does not need the image to pin the board pose, so it
    // accepts a board as small as 2x2. Four dots is still enough for Kabsch
    // (>= 3 non-collinear points) and for the dihedral enumeration (8
    // symmetries for a square).
    BoardConfig board = square_board();
    board.n_x = 2;
    board.n_y = 2;
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const std::vector<LatticeSymmetry> symmetries = lattice_symmetries(lattice);
    ASSERT_EQ(8u, symmetries.size());

    const ::Params params = make_params(800.0, 800.0, 320.0, 240.0, 0.0, 0.0, 0.0);
    const Eigen::Matrix4d T = pose_of(rotation(0.3, -0.2, 0.0), Eigen::Vector3d(0.0, 0.0, 0.6));
    const BoardProjector projector(board.n_d);
    const std::vector<::Shape> shapes = detect(projector, lattice, T, params);

    std::vector<int> identity(lattice.size());
    for (std::size_t i = 0; i < lattice.size(); ++i) identity[i] = static_cast<int>(i);
    const std::vector<Eigen::Vector3d> external = mounted_points(lattice, identity, T);

    BoardMatchOptions options;
    const BoardMatch match = match_board_pose(lattice, external, shapes, params, kRadius,
                                              projector, options, symmetries, NULL);
    ASSERT_TRUE(match.ok) << match.reason;
    EXPECT_EQ("identity", match.symmetry_name);
    EXPECT_EQ(identity, match.permutation);
    EXPECT_LT(match.match_rms_px, 1e-6);
}

// ---------------------------------------------------------------------------
// Pose buffer
// ---------------------------------------------------------------------------

PoseSample sample_at(std::int64_t stamp, double x) {
    PoseSample sample;
    sample.stamp_ns = stamp;
    sample.frame_id = "optical";
    sample.points.push_back(Eigen::Vector3d(x, 0.0, 1.0));
    return sample;
}

TEST(PoseBuffer, ExactLookup) {
    PoseBuffer buffer;
    buffer.add(sample_at(1000000000LL, 1.0));
    buffer.add(sample_at(2000000000LL, 2.0));

    PoseSample out;
    double dt = -1.0;
    std::int64_t source = -1;
    ASSERT_TRUE(buffer.lookup(1000000000LL, 0.01, 0.1, true, &out, &dt, &source));
    EXPECT_EQ(0.0, dt);
    EXPECT_EQ(1000000000LL, source);
    EXPECT_EQ(1.0, out.points[0].x());
}

TEST(PoseBuffer, InterpolatesBetweenBrackets) {
    PoseBuffer buffer;
    buffer.add(sample_at(1000000000LL, 1.0));
    buffer.add(sample_at(2000000000LL, 3.0));

    PoseSample out;
    double dt = -1.0;
    std::int64_t source = -1;
    ASSERT_TRUE(buffer.lookup(1500000000LL, 0.6, 1.1, true, &out, &dt, &source));
    EXPECT_NEAR(2.0, out.points[0].x(), 1e-12);
    EXPECT_NEAR(0.5, dt, 1e-12);
    EXPECT_EQ(1000000000LL, source);
    EXPECT_EQ(1500000000LL, out.stamp_ns);
}

TEST(PoseBuffer, LargeGapFallsBackToNearest) {
    PoseBuffer buffer;
    buffer.add(sample_at(1000000000LL, 1.0));
    buffer.add(sample_at(2000000000LL, 3.0));

    PoseSample out;
    double dt = -1.0;
    std::int64_t source = -1;
    ASSERT_TRUE(buffer.lookup(1400000000LL, 0.6, 0.2, true, &out, &dt, &source));
    EXPECT_EQ(1.0, out.points[0].x());  // nearest is the earlier sample
    EXPECT_NEAR(0.4, dt, 1e-12);
    EXPECT_EQ(1000000000LL, source);
}

TEST(PoseBuffer, RejectsSamplesOutsideTolerance) {
    PoseBuffer buffer;
    buffer.add(sample_at(1000000000LL, 1.0));

    PoseSample out;
    EXPECT_FALSE(buffer.lookup(2000000000LL, 0.01, 0.1, true, &out, NULL, NULL));
    EXPECT_FALSE(buffer.lookup(0LL, 1.0, 0.1, true, &out, NULL, NULL));
    EXPECT_TRUE(buffer.lookup(1050000000LL, 0.1, 0.1, true, &out, NULL, NULL));
}

TEST(PoseBuffer, ReplacesSameStampAndIgnoresEmpty) {
    PoseBuffer buffer;
    buffer.add(sample_at(1000000000LL, 1.0));
    buffer.add(sample_at(1000000000LL, 2.0));
    buffer.add(PoseSample());
    EXPECT_EQ(1u, buffer.size());

    PoseSample out;
    ASSERT_TRUE(buffer.lookup(1000000000LL, 0.0, 0.0, false, &out, NULL, NULL));
    EXPECT_EQ(2.0, out.points[0].x());
}

// ---------------------------------------------------------------------------
// Config parsing
// ---------------------------------------------------------------------------

TEST(ExternalConfig, ParsesPerCameraPoseTopicsAndFrames) {
    const std::string path = temp_path("config.yaml");
    {
        std::ofstream out(path.c_str());
        out << "board:\n"
               "  type: circle\n"
               "solve:\n"
               "  pose_source: external\n"
               "external:\n"
               "  sync_tolerance_s: 0.03\n"
               "  max_gap_s: 0.2\n"
               "  ambiguous_policy: use_best\n"
               "  expected_frame_id: rig_optical\n"
               "frames:\n"
               "  budget: 10\n"
               "cameras:\n"
               "  left:\n"
               "    topic: /left/image\n"
               "    source: calibrate\n"
               "    point_pose_topic: /left/points\n"
               "    point_pose_frame_id: left_optical\n"
               "  right:\n"
               "    topic: /right/image\n"
               "    source: calibrate\n"
               "    point_pose_topic: /right/points\n"
               "output:\n"
               "  directory: /tmp/discocal_external_test_output\n";
    }

    const RigConfig rig = RigConfig::load(path);
    EXPECT_EQ("external", rig.solve.pose_source);
    EXPECT_NEAR(0.03, rig.external.sync_tolerance_s, 1e-12);
    EXPECT_NEAR(0.2, rig.external.max_gap_s, 1e-12);
    EXPECT_EQ("use_best", rig.external.ambiguous_policy);
    EXPECT_EQ("rig_optical", rig.external.expected_frame_id);
    ASSERT_EQ(2u, rig.cameras.size());
    EXPECT_EQ("/left/points", rig.cameras.at("left").point_pose_topic);
    EXPECT_EQ("left_optical", rig.cameras.at("left").point_pose_frame_id);
    EXPECT_EQ("/right/points", rig.cameras.at("right").point_pose_topic);
    std::remove(path.c_str());
}

// The shipped example is user-facing documentation; a config the parser rejects
// is worse than no example. Resolved from this source file because the test
// binary runs from the build tree.
TEST(ExternalConfig, ShippedExternalExampleParses) {
    const std::string source = __FILE__;
    const std::string suffix = "/test/test_external.cpp";
    ASSERT_GT(source.size(), suffix.size());
    const std::string path = source.substr(0, source.size() - suffix.size()) +
                             "/config/rig_example_external.yaml";

    const RigConfig rig = RigConfig::load(path);
    EXPECT_EQ("external", rig.solve.pose_source);
    EXPECT_FALSE(rig.solve.joint_refine);
    ASSERT_TRUE(rig.two_cameras());
    ASSERT_TRUE(rig.camera("left") != NULL);
    ASSERT_TRUE(rig.camera("right") != NULL);
    EXPECT_EQ("/arm/calibration_target/left_points",
              rig.camera("left")->point_pose_topic);
    EXPECT_EQ("left_optical", rig.camera("left")->point_pose_frame_id);
    EXPECT_EQ("/arm/calibration_target/right_points",
              rig.camera("right")->point_pose_topic);
    EXPECT_EQ("right_optical", rig.camera("right")->point_pose_frame_id);
    EXPECT_TRUE(rig.external.interpolate);
    EXPECT_TRUE(rig.extrinsic.present);
}

TEST(ExternalConfig, RequiresAPoseTopicForEveryCalibratedCamera) {
    const std::string path = temp_path("missing_topic.yaml");
    {
        std::ofstream out(path.c_str());
        out << "solve:\n"
               "  pose_source: external\n"
               "cameras:\n"
               "  left:\n"
               "    topic: /left/image\n"
               "    source: calibrate\n";
    }
    EXPECT_THROW(RigConfig::load(path), std::runtime_error);
    std::remove(path.c_str());
}

// External mode fits the board pose from the measured positions, so it does not
// need the image path's 3-dots-per-side floor. 2 per side is accepted; a single
// row or column remains invalid because the detector cannot order a line.
TEST(ExternalConfig, AcceptsATwoRowBoardInExternalMode) {
    const std::string path = temp_path("two_row.yaml");
    {
        std::ofstream out(path.c_str());
        out << "board:\n"
               "  type: circle\n"
               "  n_x: 2\n"
               "  n_y: 5\n"
               "solve:\n"
               "  pose_source: external\n"
               "cameras:\n"
               "  left:\n"
               "    topic: /left/image\n"
               "    source: calibrate\n"
               "    point_pose_topic: /left/points\n"
               "output:\n"
               "  directory: /tmp/discocal_external_test_output\n";
    }
    const RigConfig rig = RigConfig::load(path);
    EXPECT_EQ(2, rig.board.n_x);
    EXPECT_EQ(5, rig.board.n_y);
    EXPECT_EQ("external", rig.solve.pose_source);
    std::remove(path.c_str());
}

TEST(ExternalConfig, RejectsASingleRowEvenInExternalMode) {
    const std::string path = temp_path("one_row.yaml");
    {
        std::ofstream out(path.c_str());
        out << "board:\n"
               "  type: circle\n"
               "  n_x: 1\n"
               "  n_y: 5\n"
               "solve:\n"
               "  pose_source: external\n"
               "cameras:\n"
               "  left:\n"
               "    topic: /left/image\n"
               "    source: calibrate\n"
               "    point_pose_topic: /left/points\n"
               "output:\n"
               "  directory: /tmp/discocal_external_test_output\n";
    }
    EXPECT_THROW(RigConfig::load(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(ExternalConfig, TwoRowBoardIsStillRejectedForImagePoses) {
    const std::string path = temp_path("two_row_image.yaml");
    {
        std::ofstream out(path.c_str());
        out << "board:\n"
               "  type: circle\n"
               "  n_x: 2\n"
               "  n_y: 5\n"
               "cameras:\n"
               "  left:\n"
               "    topic: /left/image\n"
               "    source: calibrate\n"
               "output:\n"
               "  directory: /tmp/discocal_external_test_output\n";
    }
    EXPECT_THROW(RigConfig::load(path), std::runtime_error);
    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// Known-pose solve
// ---------------------------------------------------------------------------

TEST(KnownPoseSolver, RecoversIntrinsicsFromMeasuredPoses) {
    const BoardConfig board = rectangular_board();
    const std::vector<Eigen::Vector3d> lattice = board_lattice(board);
    const ::Params truth = make_params(800.0, 810.0, 322.0, 238.0, 0.2, -0.05, 0.01);
    const BoardProjector projector(board.n_d);

    // Looking at a board at ~0.6 m with tilts about both axes. Eight poses is
    // fewer than a real run but enough to separate fx, fy and the distortion.
    std::vector<KnownPoseFrame> frames;
    const double tilts[8][2] = {{0.0, 0.0},    {0.15, 0.0},  {-0.15, 0.05},
                                {0.0, 0.2},    {0.0, -0.2},  {0.1, 0.12},
                                {-0.12, -0.1}, {0.2, -0.15}};
    for (int i = 0; i < 8; ++i) {
        const double distance = 0.55 + 0.02 * i;
        const Eigen::Matrix4d T =
            pose_of(rotation(tilts[i][0], tilts[i][1], 0.15 * i),
                    Eigen::Vector3d(0.01 * (i - 3), 0.005 * (i % 3), distance));

        KnownPoseFrame frame;
        frame.stamp_ns = 1000000000LL + i * 100000000LL;
        frame.shapes = detect(projector, lattice, T, truth);
        frame.points.resize(lattice.size());
        for (std::size_t k = 0; k < lattice.size(); ++k) {
            frame.points[k] = T.topLeftCorner<3, 3>() * lattice[k] + T.topRightCorner<3, 1>();
        }
        frames.push_back(frame);
    }

    KnownPoseOptions options;
    options.image_width = 640;
    options.image_height = 480;
    options.projection_mode = 0;
    options.min_frames = 5;
    options.ceres_threads = 1;
    options.external.max_rigidity_rms_m = 0.0005;
    options.external.max_match_rms_px = 5.0;
    options.external.use_dlt_initialization = true;

    KnownPoseSolver solver(board, options);
    KnownPoseResult result;
    ASSERT_TRUE(solver.solve(frames, NULL, &result)) << result.message;

    EXPECT_TRUE(result.ok);
    EXPECT_GE(result.matched_frames, options.min_frames);
    EXPECT_GT(result.params.fx(), 0.0);
    EXPECT_GT(result.params.fy(), 0.0);
    // Loose tolerances: this asserts the solve landed on the right basin, not
    // that Ceres reproduced the generator to machine precision.
    EXPECT_NEAR(truth.fx(), result.params.fx(), 0.08 * truth.fx());
    EXPECT_NEAR(truth.fy(), result.params.fy(), 0.08 * truth.fy());
    EXPECT_NEAR(truth.cx(), result.params.cx(), 15.0);
    EXPECT_NEAR(truth.cy(), result.params.cy(), 15.0);
    EXPECT_LT(result.reprojection_error_px, 1.0);
}

}  // namespace

// catkin_add_gtest links gtest but not gtest_main, so the suite supplies its own
// entry point (same reason as test_core.cpp).
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
