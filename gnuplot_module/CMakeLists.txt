cmake_minimum_required(VERSION 3.0.2)
project(gnuplot_module VERSION 0.0.1)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

add_compile_options(-std=c++11)

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)

set(LIB_CORE gnuplot_module)
add_library(${LIB_CORE} SHARED src/gnuplot_module.cpp)

target_include_directories(${LIB_CORE}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

#############
## Install ##
#############
include(GNUInstallDirs) # for correct library locations across platforms
set(config_package_location "share/${PROJECT_NAME}/cmake") # for .cmake find-scripts installs


install(
  TARGETS ${LIB_CORE}
  EXPORT  ${LIB_CORE}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(
  DIRECTORY include/gnuplot_module/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gnuplot_module
)

# Generate the find scripts
install(
  EXPORT ${LIB_CORE}-targets
  FILE ${LIB_CORE}-targets.cmake
  NAMESPACE gnuplot_module::
  DESTINATION ${config_package_location}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gnuplot_module-config-version.cmake"
  VERSION ${gnuplot_module_VERSION}
  COMPATIBILITY SameMajorVersion
)
# install the two files in a place where cmake looks for them
install(
  FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/gnuplot_module-config.cmake"         # self-written
    "${CMAKE_CURRENT_BINARY_DIR}/gnuplot_module-config-version.cmake" # generated
  DESTINATION ${config_package_location}
)
# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})