cmake_minimum_required(VERSION 3.3)
project(filter_position_optimization)
set(CMAKE_CXX_STNDARD 17)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

add_definitions(-DRESULT_DIR="${CMAKE_SOURCE_DIR}")

include_directories(include)

find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(NLOPT REQUIRED)
include_directories(${NLOPT_INCLUDE_DIRS})

find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${osqp_INCLUDE_DIR})

if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

add_executable(filter_position_opt main.cpp
                                   src/interpolate.cpp
                                   src/solver/qp_solver_gurobi.cpp
                                   src/solver/qp_solver_osqp.cpp
                                   src/solver/lp_solver_gurobi.cpp
                                   src/solver/lp_solver_osqp.cpp
                                   src/solver/nc_solver_nlopt.cpp
                                   src/solver/osqp_interface/csc_matrix_conv.cpp
                                   src/solver/osqp_interface/osqp_interface.cpp
                                   src/filter.cpp
                                   src/utils.cpp
                                   src/optimizer.cpp
                                   src/scenario_generator.cpp)

target_link_libraries(filter_position_opt PRIVATE osqp::osqp)

if(CXX)
    target_link_libraries(filter_position_opt PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(filter_position_opt PUBLIC ${GUROBI_LIBRARY} ${NLOPT_LIBRARIES})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    include(FeatureSummary)
    feature_summary(WHAT ALL)
endif()