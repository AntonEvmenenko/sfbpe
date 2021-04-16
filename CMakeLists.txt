cmake_minimum_required(VERSION 3.7)
project(ngn)

find_package(SDL2 REQUIRED)
include_directories(ngn ${SDL2_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(ngn ${EIGEN3_INCLUDE_DIR})

add_executable(ngn 
    src/main.cpp 
    src/box.h
    src/box.cpp
    src/constraint.h
    src/drawing.h
    src/drawing.cpp
    src/solver.h
    src/solver.cpp
)
target_link_libraries(ngn ${SDL2_LIBRARIES})
