# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.10)
project(simple_skelton_streamer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(../sample_helper_libs/window_controller_3d window_controller_3d)

add_executable(simple_skelton_streamer
       src/main.cpp
       src/Addition.cpp
       src/AngleCalculator.cpp
       src/camera_interface.cpp
       src/skeleton_processor.cpp
       )

target_include_directories(simple_skelton_streamer PRIVATE ../sample_helper_includes)

# インクルードディレクトリの設定
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

#find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Dependencies of this library
target_link_libraries(simple_skelton_streamer PRIVATE 
    k4a
    k4abt
    k4arecord
    window_controller_3d::window_controller_3d
    glfw
    dl
    )
