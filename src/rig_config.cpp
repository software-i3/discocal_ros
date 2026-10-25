#include "discocal_ros/rig_config.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace discocal_ros {
namespace {

// yaml-cpp 0.5 (the Ubuntu 18.04 package) needs a non-const Node to call
// as<T>(), so every read goes through a copy. Returns false for a missing key,
// a null node, or a value that will not convert.
template <typename T>
bool read_opt(const YAML::Node& parent, const char* key, T* out) {
    if (!parent) return false;
    YAML::Node node = parent[key];
    if (!node) return false;
    try {
        *out = node.as<T>();
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

// A required key. `context` names the section so the error says where to look.
template <typename T>
T read_req(const YAML::Node& parent, const char* key, const std::string& context) {
    T value;
    if (!read_opt<T>(parent, key, &value)) {
        std::ostringstream oss;
        oss << "rig config: missing or malformed '" << key << "' in " << context;
        throw std::runtime_error(oss.str());
    }
    return value;
}

// The .dwecal format's resolution byte is an enum, not a size. Keep this in
// step with dwecal_gen.cpp; the check here catches a bad config before any
// collection starts, and the generator checks again before writing.
bool dwecal_resolution_supported(int width, int height) {
    return (width == 1600 && height == 1200) || (width == 1280 && height == 720) ||
           (width == 800 && height == 600) || (width == 640 && height == 480);
}

std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    const std::size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    const std::size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

// A Params from a YAML block with the library's own field names (fx,fy,cx,cy,
// skew,d1..d4). Spelled out rather than using Params(YAML::Node)'s implicit
// conversion so the failure is a clear config error instead of a YAML
// exception surfacing from inside a constructor.
::Params params_from_node(const YAML::Node& node, const std::string& context) {
    ::Params p;
    p.K[0] = read_req<double>(node, "fx", context);
    p.K[1] = read_req<double>(node, "fy", context);
    p.K[2] = read_req<double>(node, "cx", context);
    p.K[3] = read_req<double>(node, "cy", context);
    read_opt<double>(node, "skew", &p.K[4]);  // optional; 0 is a fine default
    p.d[0] = read_req<double>(node, "d1", context);
    p.d[1] = read_req<double>(node, "d2", context);
    read_opt<double>(node, "d3", &p.d[2]);
    read_opt<double>(node, "d4", &p.d[3]);
    p.initialize_unc();
    return p;
}

}  // namespace

Eigen::Matrix3d rotation_from_rxryrz(double rx, double ry, double rz) {
    // exp(w^), the Rodrigues map: rx/ry/rz is the rotation vector, not an Euler
    // triple. See the declaration for why that distinction is not cosmetic.
    const Eigen::Vector3d w(rx, ry, rz);
    const double theta = w.norm();
    if (theta == 0.0) return Eigen::Matrix3d::Identity();
    const Eigen::AngleAxisd angle_axis(theta, w / theta);
    return angle_axis.toRotationMatrix();
}

RigConfig RigConfig::load(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("rig config: cannot read '" + path + "': " + e.what());
    }
    if (!root || !root.IsMap()) {
        throw std::runtime_error("rig config: '" + path + "' is not a YAML map");
    }

    RigConfig cfg;

    // --- board -------------------------------------------------------------
    if (const YAML::Node board = root["board"]) {
        read_opt<std::string>(board, "type", &cfg.board.type);
        read_opt<int>(board, "n_x", &cfg.board.n_x);
        read_opt<int>(board, "n_y", &cfg.board.n_y);
        read_opt<bool>(board, "asymmetric", &cfg.board.asymmetric);
        read_opt<double>(board, "radius", &cfg.board.radius);
        read_opt<double>(board, "distance", &cfg.board.distance);
        read_opt<int>(board, "n_d", &cfg.board.n_d);
        read_opt<std::string>(board, "detection_mode", &cfg.board.detection_mode);
        read_opt<bool>(board, "edge_check", &cfg.board.edge_check);
    }
    if (cfg.board.type != "circle" && cfg.board.type != "square") {
        throw std::runtime_error("rig config: board.type must be \"circle\" or \"square\", got \"" +
                                 cfg.board.type + "\"");
    }
    // Calibrator::calibrate walks `ds[0..n_d]` of a 5-element table.
    if (cfg.board.n_d < 1 || cfg.board.n_d > 4) {
        throw std::runtime_error("rig config: board.n_d must be between 1 and 4");
    }
    if (!(cfg.board.radius > 0.0) || !(cfg.board.distance > 0.0)) {
        throw std::runtime_error("rig config: board.radius and board.distance must be positive");
    }

    // --- detector ----------------------------------------------------------
    if (const YAML::Node detector = root["detector"]) {
        read_opt<double>(detector, "size_threshold_px2", &cfg.detector.size_threshold_px2);
        read_opt<double>(detector, "dist_threshold_px", &cfg.detector.dist_threshold_px);
        read_opt<int>(detector, "max_candidates", &cfg.detector.max_candidates);
        read_opt<double>(detector, "edge_contrast", &cfg.detector.edge_contrast);
        read_opt<double>(detector, "edge_inner_uniformity", &cfg.detector.edge_inner_uniformity);
        read_opt<double>(detector, "edge_outer_uniformity", &cfg.detector.edge_outer_uniformity);
        read_opt<bool>(detector, "probe", &cfg.detector.probe);
        read_opt<bool>(detector, "edge_stats", &cfg.detector.edge_stats);
    }
    if (cfg.detector.size_threshold_px2 < 0.0 || cfg.detector.dist_threshold_px < 0.0 ||
        cfg.detector.max_candidates < 0 || cfg.detector.edge_contrast < 0.0 ||
        cfg.detector.edge_inner_uniformity < 0.0 || cfg.detector.edge_outer_uniformity < 0.0) {
        throw std::runtime_error(
            "rig config: detector thresholds and max_candidates cannot be negative "
            "(0 means \"leave the library default\")");
    }

    // --- performance -------------------------------------------------------
    if (const YAML::Node perf = root["performance"]) {
        read_opt<bool>(perf, "sparse_info", &cfg.performance.sparse_info);
        read_opt<bool>(perf, "block_cov", &cfg.performance.block_cov);
        read_opt<bool>(perf, "sparse_schur", &cfg.performance.sparse_schur);
        read_opt<bool>(perf, "unc_dump", &cfg.performance.unc_dump);
        read_opt<bool>(perf, "time", &cfg.performance.time);
        read_opt<int>(perf, "detector_threads", &cfg.performance.detector_threads);
        read_opt<int>(perf, "ceres_threads", &cfg.performance.ceres_threads);
        read_opt<int>(perf, "opencv_threads", &cfg.performance.opencv_threads);
        read_opt<int>(perf, "nice", &cfg.performance.nice);
        read_opt<int>(perf, "async_spinner_threads", &cfg.performance.async_spinner_threads);
        read_opt<bool>(perf, "collect_during_solve", &cfg.performance.collect_during_solve);
    }
    if (cfg.performance.detector_threads < 0) {
        throw std::runtime_error("rig config: performance.detector_threads cannot be negative");
    }
    if (cfg.performance.ceres_threads < 1) {
        throw std::runtime_error("rig config: performance.ceres_threads must be at least 1");
    }
    if (cfg.performance.opencv_threads < 1) {
        throw std::runtime_error("rig config: performance.opencv_threads must be at least 1");
    }
    if (cfg.performance.async_spinner_threads < 1) {
        throw std::runtime_error(
            "rig config: performance.async_spinner_threads must be at least 1");
    }
    if (cfg.performance.nice < -20 || cfg.performance.nice > 19) {
        throw std::runtime_error("rig config: performance.nice must be in [-20, 19]");
    }
    // sparse_info and block_cov are two uncertainty-accumulation paths, and
    // block_cov is only consulted when sparse_info is off, so setting both is
    // meaningful rather than contradictory. No cross-validation here.

    // --- solve -------------------------------------------------------------
    if (const YAML::Node solve = root["solve"]) {
        read_opt<std::string>(solve, "pose_source", &cfg.solve.pose_source);
        read_opt<bool>(solve, "joint_refine", &cfg.solve.joint_refine);
        read_opt<bool>(solve, "joint_refine_fallback", &cfg.solve.joint_refine_fallback);
    }
    cfg.solve.pose_source = trim(cfg.solve.pose_source);
    if (cfg.solve.pose_source != "image" && cfg.solve.pose_source != "external") {
        throw std::runtime_error("rig config: solve.pose_source must be \"image\" or "
                                 "\"external\", got \"" +
                                 cfg.solve.pose_source + "\"");
    }
    // The grid-size floor depends on where the board pose comes from. With
    // pose_source image, the pose is estimated from the dots themselves, and a
    // 2xN board leaves the rotation about the long axis poorly constrained, so
    // both sides need at least 3. With pose_source external the pose is fitted
    // from measured 3D positions, so the image does not have to constrain it
    // and 2 per side is enough. It is also the floor regardless of source:
    // CircleGridFinder needs a four-corner convex hull to order the dots
    // (CCircleGridFinder.cpp:92-100), so a single row or column is a line it
    // cannot order, and Kabsch needs three non-collinear points. A 2x2 board
    // is the smallest lattice that satisfies both.
    if (cfg.solve.pose_source == "external") {
        if (cfg.board.n_x < 2 || cfg.board.n_y < 2) {
            throw std::runtime_error(
                "rig config: with solve.pose_source \"external\", board.n_x and board.n_y "
                "must both be at least 2 (a single row or column is not a detectable grid)");
        }
    } else if (cfg.board.n_x < 3 || cfg.board.n_y < 3) {
        throw std::runtime_error(
            "rig config: with solve.pose_source \"image\", board.n_x and board.n_y must "
            "both be at least 3");
    }

    // --- external pose source ---------------------------------------------
    if (const YAML::Node ext = root["external"]) {
        read_opt<double>(ext, "sync_tolerance_s", &cfg.external.sync_tolerance_s);
        read_opt<double>(ext, "max_gap_s", &cfg.external.max_gap_s);
        read_opt<bool>(ext, "interpolate", &cfg.external.interpolate);
        read_opt<double>(ext, "max_rigidity_rms_m", &cfg.external.max_rigidity_rms_m);
        read_opt<double>(ext, "max_match_rms_px", &cfg.external.max_match_rms_px);
        read_opt<double>(ext, "min_match_margin_px", &cfg.external.min_match_margin_px);
        read_opt<double>(ext, "temporal_margin_px", &cfg.external.temporal_margin_px);
        read_opt<std::string>(ext, "ambiguous_policy", &cfg.external.ambiguous_policy);
        read_opt<int>(ext, "max_em_iterations", &cfg.external.max_em_iterations);
        read_opt<bool>(ext, "use_dlt_initialization",
                       &cfg.external.use_dlt_initialization);
        read_opt<bool>(ext, "use_covariance_weights",
                       &cfg.external.use_covariance_weights);
        read_opt<std::string>(ext, "expected_frame_id", &cfg.external.expected_frame_id);
    }
    cfg.external.ambiguous_policy = trim(cfg.external.ambiguous_policy);
    cfg.external.expected_frame_id = trim(cfg.external.expected_frame_id);
    if (cfg.external.ambiguous_policy != "skip" &&
        cfg.external.ambiguous_policy != "use_best") {
        throw std::runtime_error("rig config: external.ambiguous_policy must be \"skip\" or "
                                 "\"use_best\", got \"" +
                                 cfg.external.ambiguous_policy + "\"");
    }
    if (!(cfg.external.sync_tolerance_s > 0.0)) {
        throw std::runtime_error("rig config: external.sync_tolerance_s must be positive");
    }
    if (!(cfg.external.max_gap_s > 0.0)) {
        throw std::runtime_error("rig config: external.max_gap_s must be positive");
    }
    if (!(cfg.external.max_rigidity_rms_m > 0.0)) {
        throw std::runtime_error("rig config: external.max_rigidity_rms_m must be positive");
    }
    if (!(cfg.external.max_match_rms_px > 0.0)) {
        throw std::runtime_error("rig config: external.max_match_rms_px must be positive");
    }
    if (cfg.external.min_match_margin_px < 0.0) {
        throw std::runtime_error(
            "rig config: external.min_match_margin_px cannot be negative");
    }
    if (cfg.external.temporal_margin_px < 0.0) {
        throw std::runtime_error(
            "rig config: external.temporal_margin_px cannot be negative");
    }
    if (cfg.external.max_em_iterations < 1) {
        throw std::runtime_error("rig config: external.max_em_iterations must be at least 1");
    }

    // --- frames ------------------------------------------------------------
    if (const YAML::Node frames = root["frames"]) {
        read_opt<int>(frames, "budget", &cfg.frames.budget);
        read_opt<double>(frames, "max_duration_s", &cfg.frames.max_duration_s);
        read_opt<bool>(frames, "diversity_gate", &cfg.frames.diversity_gate);
        read_opt<double>(frames, "diversity_threshold", &cfg.frames.diversity_threshold);
        read_opt<double>(frames, "holdout_fraction", &cfg.frames.holdout_fraction);
    }
    if (cfg.frames.budget < 1) {
        throw std::runtime_error("rig config: frames.budget must be at least 1");
    }
    if (cfg.frames.diversity_threshold < 0.0) {
        throw std::runtime_error("rig config: frames.diversity_threshold cannot be negative");
    }
    if (cfg.frames.holdout_fraction < 0.0 || cfg.frames.holdout_fraction >= 1.0) {
        throw std::runtime_error("rig config: frames.holdout_fraction must be in [0, 1)");
    }

    // --- check -------------------------------------------------------------
    if (const YAML::Node check = root["check"]) {
        read_opt<bool>(check, "enabled", &cfg.check.enabled);
        read_opt<double>(check, "max_rms_px", &cfg.check.max_rms_px);
        read_opt<int>(check, "retries", &cfg.check.retries);
    }
    if (cfg.check.max_rms_px <= 0.0) {
        throw std::runtime_error("rig config: check.max_rms_px must be positive");
    }
    if (cfg.check.retries < 0) {
        throw std::runtime_error("rig config: check.retries cannot be negative");
    }

    // --- cameras -----------------------------------------------------------
    const YAML::Node cameras = root["cameras"];
    if (!cameras || !cameras.IsMap() || cameras.size() == 0) {
        throw std::runtime_error("rig config: 'cameras' must be a non-empty map");
    }
    for (YAML::const_iterator it = cameras.begin(); it != cameras.end(); ++it) {
        CameraConfig cam;
        cam.name = it->first.as<std::string>();
        const YAML::Node node = it->second;
        const std::string context = "cameras." + cam.name;

        cam.topic = read_req<std::string>(node, "topic", context);
        read_opt<std::string>(node, "transport", &cam.transport);
        read_opt<std::string>(node, "frame_id", &cam.frame_id);
        read_opt<std::string>(node, "source", &cam.source);
        read_opt<std::string>(node, "point_pose_topic", &cam.point_pose_topic);
        cam.point_pose_topic = trim(cam.point_pose_topic);
        read_opt<std::string>(node, "point_pose_frame_id", &cam.point_pose_frame_id);
        cam.point_pose_frame_id = trim(cam.point_pose_frame_id);

        if (cam.transport != "compressed" && cam.transport != "raw") {
            throw std::runtime_error("rig config: " + context +
                                     ".transport must be \"compressed\" or \"raw\", got \"" +
                                     cam.transport + "\"");
        }
        if (cam.source != "calibrate" && cam.source != "file") {
            throw std::runtime_error("rig config: " + context +
                                     ".source must be \"calibrate\" or \"file\", got \"" +
                                     cam.source + "\"");
        }
        if (const YAML::Node intr = node["intrinsics"]) {
            cam.intrinsics = params_from_node(intr, context + ".intrinsics");
            cam.has_intrinsics = true;
        }
        // A camera declared `file` with no numbers anywhere is always a
        // mistake, and it would otherwise surface much later as the check
        // quietly not running.
        if (cam.source == "file" && !cam.has_intrinsics) {
            throw std::runtime_error("rig config: " + context +
                                     ".source is \"file\" but no `intrinsics` block was given");
        }

        cfg.cameras[cam.name] = cam;
    }

    // External-pose mode needs a pose feed for every camera this node is asked
    // to solve. A `file` camera is not solved, but it is the other half of the
    // cross-camera check; without its pose feed that check can only fall back
    // to an unchecked goal, so say so at load time rather than after a
    // collection. (Not fatal: allow_unchecked is a legitimate way to run.)
    if (cfg.solve.pose_source == "external") {
        for (std::map<std::string, CameraConfig>::const_iterator it = cfg.cameras.begin();
             it != cfg.cameras.end(); ++it) {
            if (it->second.source == "calibrate" && it->second.point_pose_topic.empty()) {
                throw std::runtime_error(
                    "rig config: solve.pose_source is \"external\" but cameras." + it->first +
                    ".point_pose_topic is empty; every calibrated camera needs a PoseArray "
                    "feed of its dots in its optical frame");
            }
        }
    }

    // --- extrinsic ---------------------------------------------------------
    if (const YAML::Node ext = root["extrinsic"]) {
        cfg.extrinsic.present = true;
        cfg.extrinsic.parent = read_req<std::string>(ext, "parent", "extrinsic");
        cfg.extrinsic.child = read_req<std::string>(ext, "child", "extrinsic");

        if (!cfg.camera(cfg.extrinsic.parent)) {
            throw std::runtime_error("rig config: extrinsic.parent \"" + cfg.extrinsic.parent +
                                     "\" is not a camera in `cameras`");
        }
        if (!cfg.camera(cfg.extrinsic.child)) {
            throw std::runtime_error("rig config: extrinsic.child \"" + cfg.extrinsic.child +
                                     "\" is not a camera in `cameras`");
        }
        if (cfg.extrinsic.parent == cfg.extrinsic.child) {
            throw std::runtime_error("rig config: extrinsic.parent and extrinsic.child are both \"" +
                                     cfg.extrinsic.parent + "\"");
        }

        double rx = 0.0, ry = 0.0, rz = 0.0;
        if (const YAML::Node rot = ext["rotation"]) {
            read_opt<double>(rot, "rx", &rx);
            read_opt<double>(rot, "ry", &ry);
            read_opt<double>(rot, "rz", &rz);
        }
        double tx = 0.0, ty = 0.0, tz = 0.0;
        if (const YAML::Node trans = ext["translation"]) {
            read_opt<double>(trans, "x", &tx);
            read_opt<double>(trans, "y", &ty);
            read_opt<double>(trans, "z", &tz);
        }
        cfg.extrinsic.R = rotation_from_rxryrz(rx, ry, rz);
        cfg.extrinsic.t = Eigen::Vector3d(tx, ty, tz);
    }

    // --- output ------------------------------------------------------------
    if (const YAML::Node output = root["output"]) {
        read_opt<std::string>(output, "directory", &cfg.output.directory);
    }
    cfg.output.directory = trim(cfg.output.directory);
    if (cfg.output.directory.empty()) {
        throw std::runtime_error("rig config: output.directory is empty");
    }
    // The calibrator opens `<dir>/intrinsic_parameters.yaml` unconditionally, so
    // a trailing slash is normalised away here and re-added where needed.
    while (cfg.output.directory.size() > 1 && cfg.output.directory.back() == '/') {
        cfg.output.directory.pop_back();
    }

    // --- dwecal ------------------------------------------------------------
    if (const YAML::Node dwecal = root["dwecal"]) {
        read_opt<bool>(dwecal, "enabled", &cfg.dwecal.enabled);
        read_opt<int>(dwecal, "output_width", &cfg.dwecal.output_width);
        read_opt<int>(dwecal, "output_height", &cfg.dwecal.output_height);
        read_opt<std::string>(dwecal, "basename", &cfg.dwecal.basename);
        read_opt<bool>(dwecal, "require_camera1_right", &cfg.dwecal.require_camera1_right);
    }
    cfg.dwecal.basename = trim(cfg.dwecal.basename);
    if (cfg.dwecal.basename.empty()) {
        throw std::runtime_error("rig config: dwecal.basename is empty");
    }
    if (cfg.dwecal.basename.find('/') != std::string::npos) {
        throw std::runtime_error(
            "rig config: dwecal.basename must be a file name, not a path (no '/')");
    }
    if (!dwecal_resolution_supported(cfg.dwecal.output_width, cfg.dwecal.output_height)) {
        std::ostringstream oss;
        oss << "rig config: dwecal resolution " << cfg.dwecal.output_width << "x"
            << cfg.dwecal.output_height
            << " is not representable; use 1600x1200, 1280x720, 800x600 or 640x480";
        throw std::runtime_error(oss.str());
    }

    return cfg;
}

const CameraConfig* RigConfig::camera(const std::string& name) const {
    const std::map<std::string, CameraConfig>::const_iterator it = cameras.find(name);
    return it == cameras.end() ? NULL : &it->second;
}

std::string RigConfig::other_camera(const std::string& name) const {
    if (cameras.size() != 2) return "";
    for (std::map<std::string, CameraConfig>::const_iterator it = cameras.begin();
         it != cameras.end(); ++it) {
        if (it->first != name) return it->first;
    }
    return "";
}

}  // namespace discocal_ros
