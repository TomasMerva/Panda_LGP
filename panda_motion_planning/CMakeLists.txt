cmake_minimum_required(VERSION 3.0.2)
project(panda_motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include/panda_motion_planning
  LIBRARIES panda_ik_analytic
  CATKIN_DEPENDS 
    roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(panda_motion_planning src/test.cpp )
target_link_libraries(panda_motion_planning
                      ${catkin_LIBRARIES}
                      ${EIGEN3_LIBRARIES}
                      )