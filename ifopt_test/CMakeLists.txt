cmake_minimum_required(VERSION 3.0.2)
project(ifopt_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ifopt
  roscpp
)

catkin_package(
  INCLUDE_DIRS include/ifopt_test
  LIBRARIES tools_function
  CATKIN_DEPENDS
    roscpp
#  DEPENDS system_lib
)

find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
include_directories("/home/tomas/Repos/gnuplot-iostream/gnuplot-iostream")

add_library(test_dec_variable src/test_dec_variable.cpp)
target_link_libraries(test_dec_variable ${catkin_LIBRARIES})

add_library(test_constraints src/test_constraints.cpp)
target_link_libraries(test_constraints ${catkin_LIBRARIES})

add_library(test_objective src/test_objective.cpp)
target_link_libraries(test_objective ${catkin_LIBRARIES})

add_executable(manipulator src/manipulator.cpp )
target_link_libraries(manipulator
                    test_dec_variable
                    test_constraints
                    test_objective
                    ${catkin_LIBRARIES}
                    ${Boost_LIBRARIES})

add_executable(main src/main.cpp )
target_link_libraries(main
                    test_dec_variable
                    test_constraints
                    test_objective
                    ${catkin_LIBRARIES}
                    ${Boost_LIBRARIES})

add_executable(ex_test_ipopt src/ex_test_ipopt.cc)
target_link_libraries(ex_test_ipopt
                    ${catkin_LIBRARIES}
                    ${Boost_LIBRARIES})