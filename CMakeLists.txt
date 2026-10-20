cmake_minimum_required(VERSION 3.10)
project(discocal_ros)

# C++17, not C++20.
#
# Upstream discocal sets CXX_STANDARD 20, but nothing in the vendored sources
# needs it -- all six library TUs compile clean under -std=c++17. It matters
# because the vehicle's CMake is 3.10.2 (Ubuntu 18.04), which *rejects the value*
# "20" at generate time with an error that says nothing about the C++ standard.
#
# CMAKE_CXX_EXTENSIONS is deliberately left at its default (ON) so this builds as
# -std=gnu++17, matching axis_grasp and upstream. Turning it off defines
# __STRICT_ANSI__, which changes which POSIX declarations glibc exposes to code
# that includes <unistd.h> and <sys/time.h>.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Exactly ONE find_package(catkin ...) call.
#
# A second call *replaces* catkin_INCLUDE_DIRS instead of adding to it, silently
# dropping the ROS include directories; the failure surfaces much later as
# "ros/ros.h: No such file or directory". Compose the component list here and
# call it once. axis_grasp/CMakeLists.txt carries the same note for the same
# reason.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    actionlib
    actionlib_msgs
    message_generation)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# Ceres is a hard requirement -- the vendored calibrator, the joint refine and
# every uncertainty path link it. QUIET + an explicit message for the same
# reason as yaml-cpp below: CMake's own "By not providing FindCeres.cmake"
# text names the package but not the apt package that provides it, and on a
# clean machine this is the first failure a new checkout hits. Ceres's config
# in turn requires glog, so name both. Ceres_FOUND is what CMake sets; the
# config also mirrors it as CERES_FOUND (CeresConfig.cmake lines 42-75), so
# accept either spelling rather than depend on which one a distro ships.
find_package(Ceres QUIET)
if(NOT Ceres_FOUND AND NOT CERES_FOUND)
    message(FATAL_ERROR
        "discocal_ros: Ceres was not found (the vendored calibrator needs it). "
        "On Ubuntu 18.04: apt install libceres-dev libgoogle-glog-dev")
endif()

# yaml-cpp: the vendored utils.h includes <yaml-cpp/yaml.h>, and rig_config.cpp
# parses the rig file with it. Not a catkin dependency, and it does not come in
# through any of the ones above.
#
# On Melodic the system package is 0.5, which is a *different ABI* from 0.6: 0.5
# is built against boost::shared_ptr and 0.6 is not. The public API shape changed
# too: Node::Scalar() is an inline const std::string& in 0.5 and an out-of-line
# std::string in 0.6, node_data::empty_scalar is a data object in 0.5 and a
# function in 0.6, and BadConversion's destructor is implicit in 0.5 and
# out-of-line in 0.6. Compiling against one copy's headers and linking the
# other's library is therefore a hard failure, not a runtime surprise:
#
#   undefined reference to `YAML::BadConversion::~BadConversion()'
#   undefined reference to `YAML::detail::node_data::empty_scalar[abi:cxx11]()'
#
# That is exactly what a vehicle with 0.6 headers in /usr/local/include and the
# system 0.5 library produced. No C++ package can make 0.5 and 0.6 compatible;
# what this build can do is use ONE copy for both headers and library. The
# config find_package selects is authoritative, and its headers are staged into
# this build tree below, because a stray 0.6 tree in /usr/local/include won the
# header search while the linker still bound the configured 0.5 library.
#
# QUIET + an explicit message rather than REQUIRED: the interesting part of this
# failure is the apt package name, which CMake's own message does not say.
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    message(FATAL_ERROR
        "discocal_ros: yaml-cpp was not found. On Ubuntu 18.04: apt install libyaml-cpp-dev")
endif()

# Resolve the include directory and library that belong to the config CMake
# found. 0.5 exports YAML_CPP_INCLUDE_DIR and YAML_CPP_LIBRARIES; 0.6+ may
# export YAML_CPP_INCLUDE_DIRS and/or a namespaced imported target. Take the
# first entry that actually contains yaml-cpp/yaml.h: the plural variable is a
# list, so treating it as a single path fails on the very layouts this is meant
# to handle.
set(_yaml_cpp_include_dir "")
foreach(_yaml_cpp_candidate IN LISTS YAML_CPP_INCLUDE_DIR YAML_CPP_INCLUDE_DIRS)
    if(EXISTS "${_yaml_cpp_candidate}/yaml-cpp/yaml.h")
        set(_yaml_cpp_include_dir "${_yaml_cpp_candidate}")
        break()
    endif()
endforeach()
if(NOT _yaml_cpp_include_dir)
    message(FATAL_ERROR
        "discocal_ros: the yaml-cpp CMake config was found but did not provide a usable "
        "include directory (YAML_CPP_INCLUDE_DIR='${YAML_CPP_INCLUDE_DIR}'; "
        "YAML_CPP_INCLUDE_DIRS='${YAML_CPP_INCLUDE_DIRS}'). Choose the installation "
        "explicitly with -Dyaml-cpp_DIR=<prefix>/lib/cmake/yaml-cpp.")
endif()

if(TARGET yaml-cpp::yaml-cpp)
    set(_yaml_cpp_library yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(_yaml_cpp_library yaml-cpp)
elseif(YAML_CPP_LIBRARIES)
    set(_yaml_cpp_library ${YAML_CPP_LIBRARIES})
else()
    message(FATAL_ERROR
        "discocal_ros: yaml-cpp headers were found at ${_yaml_cpp_include_dir}, but its "
        "CMake config exported no library target and no YAML_CPP_LIBRARIES.")
endif()
# Stage the selected config's headers into this build tree.
#
# This is the part that actually makes 0.5 and 0.6 stop fighting, and it cannot
# be done with an include path pointing at the installation. GCC ignores -I for
# a directory that is also a standard system include directory: -I/usr/include
# is searched at its normal system position (after /usr/local/include), not at
# the -I position. So on the vehicle, adding -I to the 0.5 headers changed
# nothing and /usr/local/include/yaml-cpp (0.6, unmanaged) still supplied the
# headers while the linker bound the 0.5.2 .so. A copy in the build tree is a
# non-system directory, so the -I that names it is honoured before every
# default directory. Symlinking the directory is not enough either: GCC
# canonicalises the path and sees /usr/include again. The copy is what works.
set(_yaml_cpp_shim_root "${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-include")
file(MAKE_DIRECTORY "${_yaml_cpp_shim_root}")
file(COPY "${_yaml_cpp_include_dir}/yaml-cpp"
     DESTINATION "${_yaml_cpp_shim_root}")
if(NOT EXISTS "${_yaml_cpp_shim_root}/yaml-cpp/yaml.h")
    message(FATAL_ERROR
        "discocal_ros: failed to stage yaml-cpp headers from "
        "'${_yaml_cpp_include_dir}' into '${_yaml_cpp_shim_root}'.")
endif()
message(STATUS
    "discocal_ros: yaml-cpp headers ${_yaml_cpp_include_dir} "
    "(staged in ${_yaml_cpp_shim_root}); library ${_yaml_cpp_library}")

# The action embeds CameraCalibration and CameraProgress, so those two must be
# declared here before generate_messages() runs; it is what puts their generated
# headers on the action's include list.
add_message_files(FILES
    CameraCalibration.msg
    CameraProgress.msg)
add_action_files(FILES CalibrateCamera.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

# catkin_package(DEPENDS <pkg>) reads <pkg>_INCLUDE_DIRS and <pkg>_LIBRARIES and
# warns when neither is set -- but these two packages export their variables
# under names catkin does not guess: Ceres as CERES_INCLUDE_DIRS/CERES_LIBRARIES,
# and yaml-cpp 0.5 as the singular YAML_CPP_INCLUDE_DIR plus YAML_CPP_LIBRARIES.
# Aliasing them is not cosmetic. This package exports discocal_core, and this is
# the same trap that bit us once already at the target_link_libraries below: a
# consumer that links discocal_core needs these flags, and without the alias it
# would get undefined YAML::detail::* symbols at link time with no hint why.
set(Ceres_INCLUDE_DIRS ${CERES_INCLUDE_DIRS})
set(Ceres_LIBRARIES ${CERES_LIBRARIES})
set(yaml-cpp_INCLUDE_DIRS ${_yaml_cpp_include_dir})
set(yaml-cpp_LIBRARIES ${_yaml_cpp_library})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES discocal_core
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge actionlib actionlib_msgs
    DEPENDS OpenCV Ceres yaml-cpp)

# ---------------------------------------------------------------------------
# Vendored discocal sources (CC BY-NC-SA 4.0 -- see THIRD_PARTY_NOTICES.md).
#
# Flattened on purpose: upstream splits these across src/lib_detect and
# src/lib_calib with one CMakeLists each, and compiles CMomentsTracker.cpp and
# CLieAlgebra.cpp into *both* its calibrator and stereoCalibrator archives,
# which duplicates every MomentsTracker::/LieAlgebra:: symbol. Here each of
# those is compiled once, and both CCalibrator.cpp and CStereoCalibrator.cpp
# link against them. CStereoCalibrator.cpp is vendored for the fixed-extrinsic
# joint refinement (patched: SPARSE_SCHUR/threads/termination, and it never
# normalises a fixed extrinsic). The CImagehandler, the pybind11 module and the
# file-based drivers (mono.hpp/stereo.hpp) are left out: this package drives the
# classes directly, and those two headers define non-inline out-of-class
# functions that would collide across translation units.
# ---------------------------------------------------------------------------
add_library(discocal_vendor STATIC
    src/discocal/lib_detect/CTargetDetector.cpp
    src/discocal/lib_detect/CCircleGridFinder.cpp
    src/discocal/lib_calib/CCalibrator.cpp
    src/discocal/lib_calib/CStereoCalibrator.cpp
    src/discocal/lib_calib/CMomentsTracker.cpp
    src/discocal/lib_calib/CLieAlgebra.cpp)

target_include_directories(discocal_vendor PUBLIC
    ${_yaml_cpp_shim_root}
    src/discocal/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS})

# Keep the staged directory first even among the compile options. CMake puts
# target include directories in -I order after the options, and an imported
# yaml-cpp target could contribute its own /usr/local/include; an explicit -I
# in CXX_FLAGS is emitted ahead of both and pins the searched headers to the
# copy that matches the linked library.
target_compile_options(discocal_vendor PUBLIC "-I${_yaml_cpp_shim_root}")

# yaml-cpp belongs here rather than on discocal_core: the vendored utils.h is a
# public header and it includes <yaml-cpp/yaml.h>, so anything that touches the
# vendored API needs it. A consumer linking only discocal_core would otherwise
# get undefined YAML::detail::* symbols -- which is exactly how this was found.
target_link_libraries(discocal_vendor PUBLIC
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ${_yaml_cpp_library})

# The detector carries four OpenMP pragmas in cal_shape_cov
# (CTargetDetector.cpp:371-433) and includes <omp.h> unconditionally. Without
# this the pragmas are *silently ignored* -- no warning, just a slower detector.
# Upstream gets -fopenmp and -lgomp only from its own lib_detect/CMakeLists.txt.
if(OpenMP_CXX_FOUND)
    target_link_libraries(discocal_vendor PUBLIC OpenMP::OpenMP_CXX)
endif()

# GCC 7 and 8 keep the filesystem implementation in a separate libstdc++fs
# archive that has to be named on the link line, or the link fails on undefined
# experimental::filesystem::v1 symbols even though the compile succeeded. The
# vehicle builds with GCC 7.5. Same guard as axis_grasp's adapter_common.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(discocal_vendor PUBLIC stdc++fs)
endif()

# ---------------------------------------------------------------------------
# ROS-free core: config, diversity gate, calibration session, cross-camera
# check, camera_info writer. No ROS headers, so it unit-tests standalone.
# ---------------------------------------------------------------------------
add_library(discocal_core STATIC
    src/rig_config.cpp
    src/diversity_gate.cpp
    src/board_frame.cpp
    src/cross_camera_check.cpp
    src/camera_info_writer.cpp
    src/calibrate_session.cpp
    src/rig_session.cpp
    src/performance_report.cpp
    src/dwecal_gen.cpp
    src/dwecal_bridge.cpp
    src/sha256.cpp)

target_include_directories(discocal_core PUBLIC include)
# Deliberately no ${catkin_LIBRARIES}: discocal_core is ROS-free, and linking the
# ROS libraries into it would quietly make a ROS header compile here when it
# should not. It is also what would hide the include problem below.
target_link_libraries(discocal_core PUBLIC discocal_vendor)

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
add_executable(discocal_ros_node
    src/node.cpp
    src/image_source.cpp)

# ${catkin_INCLUDE_DIRS} has to be named explicitly. In this catkin version
# ${catkin_LIBRARIES} is a list of library paths rather than imported targets, so
# it carries no INTERFACE include directories -- nothing propagates them, and
# without this the compile fails on "ros/ros.h: No such file or directory" even
# though the link line would have been correct.
target_include_directories(discocal_ros_node PRIVATE ${catkin_INCLUDE_DIRS})

target_link_libraries(discocal_ros_node
    discocal_core
    ${catkin_LIBRARIES})

add_dependencies(discocal_ros_node ${PROJECT_NAME}_generate_messages_cpp)

# ---------------------------------------------------------------------------
# .dwecal generator CLI (no ROS): reproduces the node's file from a run's
# land/ directory, and is what the golden tests compare against.
# ---------------------------------------------------------------------------
add_executable(dwecal_gen_cli src/dwecal_cli.cpp)
target_link_libraries(dwecal_gen_cli discocal_core)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
find_package(GTest QUIET)
if(GTest_FOUND OR GTEST_FOUND)
    catkin_add_gtest(discocal_core_tests test/test_core.cpp)
    if(TARGET discocal_core_tests)
        target_link_libraries(discocal_core_tests discocal_core)
    endif()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS discocal_ros_node dwecal_gen_cli
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# The scripts go through catkin_install_python rather than install(DIRECTORY) so
# they land in lib/discocal_ros/ where rosrun looks for an executable, with the
# shebang rewritten to this system's python. install(DIRECTORY) would instead
# scatter them into share/ and preserve the source mode, which works from a devel
# space only because rosrun falls back to searching the source directory -- and
# silently stops working the moment the executable bit is lost, or the workspace
# is installed rather than devel.
catkin_install_python(PROGRAMS
    scripts/publish_image_dir.py
    scripts/calibrate_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
