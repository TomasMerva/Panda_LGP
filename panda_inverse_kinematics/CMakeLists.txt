cmake_minimum_required(VERSION 3.0.2)
project(panda_inverse_kinematics)

add_compile_options(-std=c++17)

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

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
    panda_gazebo_controllers 
    roscpp
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

add_library(ik_lib SHARED
  src/inverse_kinematics.cpp
)
target_link_libraries(ik_lib ${catkin_LIBRARIES})

add_executable(inverse_kinematics src/main.cpp)
target_link_libraries(inverse_kinematics
                      ik_lib
                      "/usr/local/lib/libnlopt.so"
                      gnuplot_module
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )