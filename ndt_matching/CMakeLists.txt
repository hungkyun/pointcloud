cmake_minimum_required(VERSION 2.8.3)
project(ndt_matching)
SET(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   nav_msgs
   tf
   pcl_ros
   sensor_msgs
   pcl_conversions
)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()
find_package(Eigen3 QUIET)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pandar
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization)
include_directories( include
  ${PCL_LIBRARY_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(include)
add_library(ndt_omp src/pclomp/voxel_grid_covariance_omp.cpp src/pclomp/ndt_omp.cpp src/pclomp/gicp_omp.cpp)
catkin_package()
add_executable(ndt_matching src/ndt_matching.cpp)
add_dependencies(ndt_matching ndt_omp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)
