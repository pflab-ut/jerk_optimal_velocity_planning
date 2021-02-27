cmake_minimum_required(VERSION 3.10)
project(filter_position_optimization)

set(CMAKE_CXX_STANDARD 14)

if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 -g -D_GLIBCXX_USE_CXX11_ABI=0")
    set(GUROBI_INCLUDE_DIRECTORY "/Library/gurobi901/mac64/include")
    set(GUROBI_LIB_DIRECTORY "/Library/gurobi901/mac64/lib")
    set(SOURCE_FILES src/main.cpp ${GUROBI_INCLUDE_DIRECTORY}/gurobi_c++.h)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 -g -D_GLIBCXX_USE_CXX11_ABI=0")
    set(GUROBI_INCLUDE_DIRECTORY "/opt/gurobi901/linux64/include")
    set(GUROBI_LIB_DIRECTORY "/opt/gurobi901/linux64/lib")
    set(SOURCE_FILES main.cpp ${GUROBI_INCLUDE_DIRECTORY}/gurobi_c++.h)
endif()

include_directories(${GUROBI_INCLUDE_DIRECTORY})
link_directories(${GUROBI_LIB_DIRECTORY})

include_directories(include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(NLOPT REQUIRED)
include_directories(${NLOPT_INCLUDE_DIRS})


add_library(FILTER_LIB src/interpolate.cpp
                       src/solver/qp_solver_gurobi.cpp
                       src/solver/lp_solver_gurobi.cpp
                       src/solver/nc_solver_nlopt.cpp
                       src/filter.cpp
                       src/utils.cpp
                       src/optimizer.cpp
                       src/scenario_generator.cpp)
target_link_libraries(FILTER_LIB gurobi_c++)
target_link_libraries(FILTER_LIB gurobi90)
if(APPLE)
    target_link_libraries(FILTER_LIB ${NLOPT_LIBRARIES})
else()
    target_link_libraries(FILTER_LIB "/usr/local/lib/libnlopt.so")
endif()

add_executable(filter_position_opt main.cpp)
target_link_libraries(filter_position_opt PRIVATE FILTER_LIB)