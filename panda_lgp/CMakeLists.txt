cmake_minimum_required(VERSION 3.0.2)
project(panda_lgp)

add_compile_options(-std=c++17)


IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

# so dynamic library libtowr.so retains link to ifopt_core.so
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

find_package(autodiff)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)
find_package(catkin REQUIRED COMPONENTS
  ifopt
  panda_gazebo_controllers
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    panda_gazebo_controllers 
    gnuplot_module
  DEPENDS
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories("/usr/local/include")
include_directories("/usr/local/include/gnuplot_module")

add_library(lgp_lib SHARED
  # constraints
  src/constraints/manipulation_frame_constraint.cpp
  src/constraints/axis_in_region_constraint.cpp
  src/constraints/z_axis_constraint.cpp
  # Logic
  src/action_skeleton/skeleton.cpp
  src/action_skeleton/operators.cpp
  # KOMO framework
  src/KOMO/komo.cpp
  src/KOMO/komo_objective.cpp
  # utils
  src/utils/kinematics.cpp
  src/utils/motion_ros_tools.cpp
)
target_link_libraries(lgp_lib 
                      "/usr/local/lib/libnlopt.so"
                      gnuplot_module 
                      autodiff::autodiff
                      ${catkin_LIBRARIES})

add_executable(panda_lgp src/main.cpp)
target_link_libraries(panda_lgp
                      lgp_lib
                      "/usr/local/lib/libnlopt.so"
                      gnuplot_module
                      autodiff::autodiff
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )