#ifndef DISCOCAL_ROS_DWECAL_BRIDGE_H_
#define DISCOCAL_ROS_DWECAL_BRIDGE_H_

// Conversion from this package's calibration results to DwecalInput.
//
// The two conventions differ by a transpose and a sign, and getting it wrong is
// silent: the maps still look like maps, the baseline is unchanged, and only
// the cross-camera residual or the disparity sign reveals it. The conversion is
// therefore a single named function with a unit test, not an inline expression
// at the call site:
//
//   rig config:    p_parent = R_config * p_child + t_config
//                  (t_config is the child's origin in the parent's frame)
//
//   stereoRectify: X_cam1 = R_land * X_cam0 + T_land
//
//   parent = camera0, child = camera1, so
//       R_land = R_config^T
//       T_land = -R_config^T * t_config
//
// Invariant used by the tests: the child's origin in the parent's frame,
// -R_land^T * T_land, is exactly t_config.

#include "discocal_ros/dwecal_gen.h"
#include "discocal_ros/rig_config.h"
#include "utils.h"  // ::Params

namespace discocal_ros {

// `parent_params` are camera0's intrinsics and `child_params` camera1's; the
// names in `extrinsic` are not consulted here, only its R and t.
DwecalInput dwecal_input_from_config(const ::Params& parent_params,
                                     const ::Params& child_params,
                                     const ExtrinsicConfig& extrinsic);

}  // namespace discocal_ros

#endif  // DISCOCAL_ROS_DWECAL_BRIDGE_H_
