cmake_minimum_required(VERSION 3.0.2)
project(panda_gazebo_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  eigen_conversions
  franka_gripper
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  message_generation
  roscpp
  rospy
  tf
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

add_message_files(FILES
  JointPosition.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_gazebo_controllers
  CATKIN_DEPENDS 
    controller_interface
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)


add_library(panda_gazebo_controllers
  src/joint_position_controller.cpp
)

add_dependencies(panda_gazebo_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  # ${PROJECT_NAME}_gencfg
)

target_link_libraries(panda_gazebo_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda_gazebo_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(panda_gazebo_controllers PUBLIC
  include
)


## Installation
install(TARGETS panda_gazebo_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES panda_gazebo_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(panda_gazebo_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(panda_gazebo_controllers
    FILES ${SOURCES}
    DEPENDS panda_gazebo_controllers
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()