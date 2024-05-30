cmake_minimum_required(VERSION 3.0.2)
project(tamp)
set(CMAKE_CXX_FLAGS "-std=c++14")
#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros 
  pcl_conversions
)
message("OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include
)

catkin_package()
add_executable(task_motion_planning src/task_motion_planning.cpp src/task_planning.cpp src/visualizer.cpp src/motion_planning.cpp)

target_link_libraries(task_motion_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
