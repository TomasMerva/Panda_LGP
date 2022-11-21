cmake_minimum_required(VERSION 3.0.2)
project(ceres_testy)

add_compile_options(-std=c++17)

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    gnuplot_module
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
# include_directories("/usr/local/include")
include_directories("/usr/local/include/gnuplot_module")

add_library(3dof_robot_lib
            src/solver.cpp
            src/komo_objective.cpp
            src/visualize_rviz.cpp
            src/collision_constraint.cpp
            src/mu_loss_function.cpp
            )
target_link_libraries(3dof_robot_lib Ceres::ceres ${catkin_LIBRARIES})


add_executable(3dof_robot_komo src/main_3dof_robot_komo.cpp)
target_link_libraries(3dof_robot_komo
                      3dof_robot_lib
                      gnuplot_module
                      Ceres::ceres
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )