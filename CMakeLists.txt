cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp pcl_ros sensor_msgs cv_bridge
)
add_definitions(-w)


set(OpenCV_DIR "/home/tonglu/github/install/opencv3.2.0/share/OpenCV")
find_package(OpenCV 3.2.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(Pangolin_DIR "/home/tonglu/github/Pangolin/install/lib/cmake/Pangolin")
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CGAL COMPONENTS Core)

set(ORB_SOURCE_DIR /home/tonglu/github/slam/ORB_SLAM3/)

include_directories(
${PROJECT_SOURCE_DIR}/ORB_SLAM3
${PROJECT_SOURCE_DIR}/ORB_SLAM3/include
${PROJECT_SOURCE_DIR}/ORB_SLAM3/include/CameraModels
${Pangolin_INCLUDE_DIRS}

${PCL_INCLUDE_DIRS} 
${catkin_INCLUDE_DIRS}
${ORB_SOURCE_DIR}/Thirdparty/utils/include
)

# orb lib
file(GLOB_RECURSE ORB_CC       ORB_SLAM3/src/*.cc)
file(GLOB_RECURSE ORB_CPP      ORB_SLAM3/src/*.cpp)
file(GLOB_RECURSE ORB_CAM_CPP  ORB_SLAM3/src/CameraModels/*.cpp)
file(GLOB_RECURSE ORB_H        ORB_SLAM3/include/*.h)
file(GLOB_RECURSE ORB_CA_H     ORB_SLAM3/include/CameraModels/*.h)
add_library(orb_debug SHARED ${ORB_CC} ${ORB_CPP} ${ORB_CAM_CPP} ${ORB_H} ${ORB_CA_H}
)
set_target_properties(orb_debug PROPERTIES LINKER_LANGUAGE CXX)

message(STATUS --------${ORB_CPP})
target_link_libraries(orb_debug
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SOURCE_DIR}/Thirdparty/utils/lib/libutils.so
-lboost_serialization
-lcrypto
${PCL_LIBRARIES}
${catkin_LIBRARIES}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
# ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
# ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
orb_debug
# ${PROJECT_SOURCE_DIR} ORB_SLAM3/lib/libORB_SLAM3.so
-lboost_system
CGAL::CGAL CGAL::CGAL_Core
)

# # Node for monocular camera
add_executable(Mono  src/ros_mono.cc)
# rosbuild_add_executable(Mono
# src/ros_mono.cc
# )

target_link_libraries(Mono
${LIBS}
)

# # Node for monocular camera (Augmented Reality Demo)
# rosbuild_add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )

# target_link_libraries(MonoAR
# ${LIBS}
# )

# # Node for stereo camera
# rosbuild_add_executable(Stereo
# src/ros_stereo.cc
# )

# target_link_libraries(Stereo
# ${LIBS}
# )

# # Node for RGB-D camera
# rosbuild_add_executable(RGBD
# src/ros_rgbd.cc
# )

# target_link_libraries(RGBD
# ${LIBS}
# )

# # Node for monocular-inertial camera
# rosbuild_add_executable(Mono_Inertial
# src/ros_mono_inertial.cc
# )

# target_link_libraries(Mono_Inertial
# ${LIBS}
# )

# # Node for stereo-inertial camera
# rosbuild_add_executable(Stereo_Inertial
# src/ros_stereo_inertial.cc
# )

# target_link_libraries(Stereo_Inertial
# ${LIBS}
# )
