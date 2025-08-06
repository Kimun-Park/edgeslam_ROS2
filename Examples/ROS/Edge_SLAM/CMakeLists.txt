cmake_minimum_required(VERSION 3.5)
project(edge_slam_ros2)

# Edge-SLAM: replaced next include statement with the content of rosbuild.cmake
# and made edits as indicated below to prevent cmake deprecated policy warnings
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
########## Start of rosbuild.cmake ##########
#cmake_minimum_required(VERSION 2.4.6)

# Edge-SLAM: added cmake version check if statement
# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
  if(${CMAKE_VERSION} VERSION_LESS "3.0")
    # Logical target names must be globally unique.
    cmake_policy(SET CMP0002 OLD)
    # Libraries linked via full path no longer produce linker search paths.
    cmake_policy(SET CMP0003 OLD)
    # Preprocessor definition values are now escaped automatically.
    cmake_policy(SET CMP0005 OLD)
    if(POLICY CMP0011)
      # Included scripts do automatic cmake_policy PUSH and POP.
      cmake_policy(SET CMP0011 OLD)
    endif(POLICY CMP0011)
  endif()
endif(COMMAND cmake_policy)

# Edge-SLAM: added to prevent cmake warning "Policy CMP0003 should be set before this line."
if(COMMAND cmake_policy)
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.0")
    cmake_policy(SET CMP0003 NEW)
  endif()
endif(COMMAND cmake_policy)

set(CMAKE_OSX_ARCHITECTURES "x86_64")

find_package(ament_cmake REQUIRED)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 4 REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost COMPONENTS system serialization filesystem REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libEdge_SLAM.so
)

# Node for monocular camera
#rosbuild_add_executable(Mono
#src/ros_mono.cc
#)

#target_link_libraries(Mono
#${LIBS}
#)

# Node for stereo camera
#rosbuild_add_executable(Stereo
#src/ros_stereo.cc
#)

#target_link_libraries(Stereo
#${LIBS}
#)

# Node for RGB-D camera
add_executable(RGBD src/ros_rgbd.cc)
ament_target_dependencies(RGBD
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  OpenCV
  Eigen3
  Pangolin
  Boost
)

target_link_libraries(RGBD
${LIBS}
)

install(TARGETS RGBD
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()