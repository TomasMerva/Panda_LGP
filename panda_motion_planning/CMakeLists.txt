cmake_minimum_required(VERSION 3.0.2)
project(panda_motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
# find_package(ifopt)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  panda_gazebo_controllers
  ifopt
)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

#simple_optim_mp
catkin_package(
  INCLUDE_DIRS include/panda_motion_planning
  LIBRARIES panda_ik_analytic panda_kinematics 
  CATKIN_DEPENDS 
    roscpp
    panda_gazebo_controllers
  DEPENDS drake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(panda_kinematics_lib src/panda_kinematics.cpp)
target_link_libraries(panda_kinematics_lib ${catkin_LIBRARIES})

add_executable(panda_motion_planning src/test.cpp )
target_link_libraries(panda_motion_planning
                      panda_kinematics_lib
                      ${catkin_LIBRARIES}
                      ${EIGEN3_LIBRARIES}
                      )

# Formulate (ifopt:ifopt_core) and solve (ifopt::ifopt_ipopt) the problem
# add_executable(ifopt_test src/ifopt_test.cpp)
# # Pull in include directories, libraries, ... 
# target_link_libraries(ifopt_test ${catkin_LIBRARIES} drake::drake)       

add_executable(drake_test src/drake_test.cpp)
# Pull in include directories, libraries, ... 
target_link_libraries(drake_test ${catkin_LIBRARIES} drake::drake)       