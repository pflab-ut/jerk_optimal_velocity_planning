cmake_minimum_required(VERSION 3.10)
project(filter_position_optimization)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(osqp REQUIRED)

add_library(FILTER_LIB src/interpolate.cpp
                       src/solver_interface/osqp_interface.cpp
                       src/solver_interface/csc_matrix_conv.cpp
                       src/filter.cpp
                       src/utils.cpp
                       src/qp_interface.cpp)
target_link_libraries(FILTER_LIB osqp::osqp)

add_executable(filter_position_optimization main.cpp)
target_link_libraries(filter_position_optimization PRIVATE FILTER_LIB)

add_executable(pseudo_optimization main_pseudo.cpp)
target_link_libraries(pseudo_optimization PRIVATE FILTER_LIB)
