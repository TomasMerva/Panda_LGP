cmake_minimum_required(VERSION 3.0.2)
project(panda_motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

# so dynamic library libtowr.so retains link to ifopt_core.so
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ifopt
  #panda_gazebo_controllers
)


#simple_optim_mp
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
 #   panda_gazebo_controllers
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories("/usr/local/include")
# include_directories("/home/tomas/Repos/gnuplot-iostream/gnuplot-iostream")


#---- KOMO--------
add_library(komo_lib src/komo_lib.cpp)
target_link_libraries(komo_lib ${catkin_LIBRARIES})

add_library(joint_variable src/variables/joints.cpp)
target_link_libraries(joint_variable ${catkin_LIBRARIES})

add_library(komo_k2_cost src/objective/komo_k2.cpp)
target_link_libraries(joint_variable ${catkin_LIBRARIES})

# SET(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")
# SET(NLOPT_LIBRARIES "/usr/local/lib/libnlopt.so.0")

add_executable(komo src/komo.cpp)
target_link_libraries(komo 
                      komo_lib
                      joint_variable
                      komo_k2_cost
                      "/usr/local/lib/libnlopt.so"
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )


add_library(panda_kinematics_lib src/panda_kinematics.cpp)
target_link_libraries(panda_kinematics_lib ${catkin_LIBRARIES})

add_executable(panda_motion_planning src/test.cpp )
target_link_libraries(panda_motion_planning
                      panda_kinematics_lib
                      ${catkin_LIBRARIES}
                      ${EIGEN3_LIBRARIES}
                      )


                      
