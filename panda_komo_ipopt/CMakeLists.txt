cmake_minimum_required(VERSION 3.22)
project(panda_komo_ipopt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

find_package(autodiff)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ifopt
  panda_gazebo_controllers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    gnuplot_module
    panda_gazebo_controllers
  DEPENDS 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)
include_directories("/usr/local/include")
include_directories("/usr/local/include/gnuplot_module")

add_library(komo_ipopt_lib SHARED
  src/komo_lib.cpp
  # decision variables
  src/variables/joints.cpp
  # objectives
  src/objectives/komo_k2.cpp
  # utils
  src/utils/motion_planning_tools.cpp
)
target_link_libraries(komo_ipopt_lib ${catkin_LIBRARIES})

add_executable(komo_ipopt src/main.cpp)
target_link_libraries(komo_ipopt
                      komo_ipopt_lib
                      "/usr/local/lib/libnlopt.so"
                      gnuplot_module
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      autodiff::autodiff
                      )
