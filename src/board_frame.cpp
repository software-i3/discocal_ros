#include "discocal_ros/board_frame.h"

#include <cmath>

#include "CMomentsTracker.h"

namespace discocal_ros {

std::vector<Eigen::Vector3d> board_lattice(const BoardConfig& board) {
    std::vector<Eigen::Vector3d> lattice;
    lattice.reserve(static_cast<std::size_t>(board.n_x) * static_cast<std::size_t>(board.n_y));

    if (board.asymmetric) {
        // Short rows offset by half a pitch. Every row holds n_y dots, so the
        // stride is n_y -- note this is the opposite of the symmetric case.
        for (int i = 0; i < board.n_x; ++i) {
            const double offset = (i % 2 == 0) ? board.distance / 2.0 : 0.0;
            for (int j = 0; j < board.n_y; ++j) {
                const double x = board.distance * i / 2.0;
                const double y = board.distance * j + offset;
                lattice.push_back(Eigen::Vector3d(x, y, 0.0));
            }
        }
    } else {
        // n_x dots per row, n_y rows. y counts down, which only flips which
        // corner is the origin; it does not matter for a check that is symmetric
        // in the board plane, but it is kept exact so this really is the
        // library's lattice and not a coincidentally equivalent one.
        for (int j = 0; j < board.n_y; ++j) {
            for (int i = 0; i < board.n_x; ++i) {
                const double x = board.distance * (i + 1);
                const double y = board.distance * (board.n_y - j);
                lattice.push_back(Eigen::Vector3d(x, y, 0.0));
            }
        }
    }

    return lattice;
}

BoardProjector::BoardProjector(int n_d) : tracker_(new MomentsTracker(n_d)) {}

BoardProjector::~BoardProjector() { delete tracker_; }

Eigen::Vector2d BoardProjector::project(double wx, double wy, double radius,
                                        const Eigen::Matrix4d& T, const ::Params& params,
                                        int mode) const {
    // MomentsTracker::project takes the *plane* homography, not a 4x4: for a
    // board whose z is 0 it is [R.col(0) R.col(1) t], which is exactly what
    // LieAlgebra::to_E builds (lib_calib/CLieAlgebra.cpp). So take columns 0, 1
    // and 3 of the 4x4 and drop column 2.
    Eigen::Matrix3d E;
    E << T(0, 0), T(0, 1), T(0, 3), T(1, 0), T(1, 1), T(1, 3), T(2, 0), T(2, 1), T(2, 3);

    const ::Point p = tracker_->project(wx, wy, radius, params, E, mode);
    return Eigen::Vector2d(p.x, p.y);
}

std::vector<Eigen::Vector2d> BoardProjector::project_all(
    const std::vector<Eigen::Vector3d>& lattice, double radius, const Eigen::Matrix4d& T,
    const ::Params& params, int mode) const {
    std::vector<Eigen::Vector2d> out;
    out.reserve(lattice.size());
    for (std::size_t i = 0; i < lattice.size(); ++i) {
        out.push_back(project(lattice[i].x(), lattice[i].y(), radius, T, params, mode));
    }
    return out;
}

double BoardProjector::reprojection_error(const std::vector<Eigen::Matrix4d>& T_board_to_camera,
                                          const std::vector<std::vector<::Shape>>& detections,
                                          const std::vector<Eigen::Vector3d>& lattice,
                                          double radius, const ::Params& params,
                                          int mode) const {
    if (T_board_to_camera.empty() || lattice.empty()) return 0.0;

    double error_total = 0.0;
    int frames = 0;
    for (std::size_t i = 0; i < T_board_to_camera.size() && i < detections.size(); ++i) {
        if (detections[i].size() != lattice.size()) continue;

        double frame_total = 0.0;
        for (std::size_t j = 0; j < lattice.size(); ++j) {
            const Eigen::Vector2d projected = project(lattice[j].x(), lattice[j].y(), radius,
                                                      T_board_to_camera[i], params, mode);
            const double du = projected.x() - detections[i][j].x;
            const double dv = projected.y() - detections[i][j].y;
            frame_total += std::sqrt(du * du + dv * dv);
        }
        error_total += frame_total / static_cast<double>(lattice.size());
        ++frames;
    }
    if (frames == 0) return 0.0;
    return error_total / static_cast<double>(frames);
}

}  // namespace discocal_ros
