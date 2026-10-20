# Build/verification environment for discocal_ros.
#
# Base is a pre-existing Melodic image (Ubuntu 18.04 / GCC 7.5 / OpenCV 3.2)
# that already carries the ROS packages package.xml lists, built locally as
# `myros:melodic`. Substitute any equivalent and adjust the tag below. A stock
# `ros:melodic-ros-base` is NOT equivalent: it has no cv_bridge (and so no
# OpenCV dev files), and this package's find_package(catkin COMPONENTS cv_bridge)
# fails before Ceres is ever looked for. On such an image run
# `rosdep install --from-paths src --ignore-src -r -y` in the workspace instead
# of relying on the apt lines below.
#
# The only thing such an image lacks for this package is Ceres and its glog
# dependency, which the vendored discocal library needs -- plus yaml-cpp, which
# the vendored utils.h includes. Baking them in keeps every later `catkin_make`
# from paying a per-run apt-get.
#
# The vehicle needs exactly the same three, and all of them are in the Ubuntu
# 18.04 archive: apt-get install libceres-dev libgoogle-glog-dev libyaml-cpp-dev
FROM myros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
        libceres-dev \
        libgoogle-glog-dev \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Ceres pulls in Eigen3 and SuiteSparse itself; verify rather than assume.
RUN test -f /usr/include/ceres/ceres.h \
 && test -f /usr/include/glog/logging.h \
 && test -f /usr/include/yaml-cpp/yaml.h \
 && echo "build env ok"
