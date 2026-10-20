#include "discocal_ros/dwecal_bridge.h"

namespace discocal_ros {

DwecalInput dwecal_input_from_config(const ::Params& parent_params,
                                     const ::Params& child_params,
                                     const ExtrinsicConfig& extrinsic) {
    DwecalInput in;

    const ::Params* params[2] = {&parent_params, &child_params};
    for (int i = 0; i < 2; ++i) {
        std::array<double, 9>& c = i == 0 ? in.camera0 : in.camera1;
        c[0] = params[i]->K[0];  // fx
        c[1] = params[i]->K[1];  // fy
        c[2] = params[i]->K[2];  // cx
        c[3] = params[i]->K[3];  // cy
        c[4] = params[i]->K[4];  // skew
        c[5] = params[i]->d[0];  // d1 / k1
        c[6] = params[i]->d[1];  // d2 / k2
        c[7] = params[i]->d[2];  // d3 -- generator refuses non-zero
        c[8] = params[i]->d[3];  // d4
    }

    // See the header for the derivation. R is row-major in DwecalInput, and
    // Eigen matrices are column-major in memory, so copy element by element
    // rather than through .data().
    const Eigen::Matrix3d R_land = extrinsic.R.transpose();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) in.R[row * 3 + col] = R_land(row, col);
    }
    const Eigen::Vector3d T_land = -R_land * extrinsic.t;
    in.T[0] = T_land.x();
    in.T[1] = T_land.y();
    in.T[2] = T_land.z();

    return in;
}

}  // namespace discocal_ros
