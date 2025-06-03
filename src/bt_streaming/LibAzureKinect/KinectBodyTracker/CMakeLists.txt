# KinectBodyTracker library CMake configuration
cmake_minimum_required(VERSION 3.10)
project(kinect_body_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ライブラリのソースファイル
set(LIB_SOURCES
    KinectBodyTracker.cpp
)

# ライブラリのヘッダーファイル
set(LIB_HEADERS
    KinectBodyTracker.h
)

# ライブラリの作成（静的ライブラリ）
add_library(kinect_body_tracker STATIC ${LIB_SOURCES} ${LIB_HEADERS})

# 必要なインクルードディレクトリ
target_include_directories(kinect_body_tracker PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 依存ライブラリ
target_link_libraries(kinect_body_tracker PUBLIC
    k4a
    k4abt
)

# エクスポート設定
set_target_properties(kinect_body_tracker PROPERTIES
    PUBLIC_HEADER "${LIB_HEADERS}"
)

add_executable(kinect_tracker_example example.cpp)
    target_link_libraries(kinect_tracker_example PRIVATE
        kinect_body_tracker)

# インストール設定
install(TARGETS kinect_body_tracker
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include/kinect_body_tracker
)
