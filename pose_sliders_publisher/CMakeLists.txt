cmake_minimum_required(VERSION 3.0.2)
project(pose_sliders_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_test_2
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set(QT_LIBRARIES Qt5::Widgets)
#include(${QT_USE_FILE})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/Sources/)
include_directories(${PROJECT_SOURCE_DIR}/Headers/)
include_directories(${PROJECT_SOURCE_DIR}/Forms/)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
SET(SOURCES ${PROJECT_SOURCE_DIR}/Sources/main.cpp
        ${PROJECT_SOURCE_DIR}/Sources/mainwindow.cpp)
SET(HEADERS ${PROJECT_SOURCE_DIR}/Headers/mainwindow.h)
SET(FORMS ${PROJECT_SOURCE_DIR}/Forms/mainwindow.ui)


QT5_WRAP_CPP(HEADERS_MOC ${HEADERS})
QT5_WRAP_UI(FORMS_HEADERS ${FORMS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${FORMS} ${HEADERS_MOC} ${FORMS_HEADERS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES})
