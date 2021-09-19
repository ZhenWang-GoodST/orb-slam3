cmake_minimum_required(VERSION 2.8.3)
project(image_node)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# add_definitions(-w)
# rosbuild_init()
# IF(NOT ROS_BUILD_TYPE)
#   SET(ROS_BUILD_TYPE Release)
# ENDIF()
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(OpenCV_DIR "/home/tonglu/github/install/opencv3.2.0/share/OpenCV")
# set(Ceres_DIR "/home/wz/github/install/ceres_docker/lib/cmake/Ceres")
find_package(OpenCV  REQUIRED)
find_package(Eigen3   REQUIRED)  
# find_package(CGAL COMPONENTS Core)
# find_package(ompl)
# find_package(PCL REQUIRED)
find_package(Ceres)

# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################
find_package(catkin REQUIRED COMPONENTS
  roscpp pcl_ros sensor_msgs
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    # /home/wz/github/install/ceres_docker/include
    )
message(STATUS ceres ------- ${CERES_INCLUDE_DIRS} )

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(GLOB_RECURSE UTILS ${CMAKE_CURRENT_SOURCE_DIR}/*cpp)
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM UTILS ${NODE_SRCS})
add_library(utils ${UTILS})
# rosbuild_add_library(utils
# ${UTILS}
# )
target_link_libraries(utils 
${CERES_LIBRARIES}
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
CGAL::CGAL CGAL::CGAL_Core
)
# add_library(optimizer STATIC 
# src/optimizer/optimizer.cpp
# )
# target_link_libraries(optimizer ${CERES_LIBRARIES})

# add_library(utils STATIC  src/opencv_utils.cpp src/vo_type_utils.cpp)
# target_link_libraries(utils ${catkin_LIBRARIES})

# # file(GLOB_RECURSE OPTIMIZER ${CMAKE_CURRENT_SOURCE_DIR}/include/optimizer/*cpp)
# add_library(computervision STATIC 
# src/sfm.cpp 
# src/computervision.cpp 
# src/homography_decomp.cpp)
# target_link_libraries(computervision ${catkin_LIBRARIES})

# add_library(photogrammetry STATIC src/photogrammetry.cpp)
# target_link_libraries(photogrammetry ${catkin_LIBRARIES})

# message(STATUS opencv ------- ${OpenCV_INCLUDE_DIRS})
