cmake_minimum_required(VERSION 3.8)
project(bt_streaming)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# KinectBodyTrackerライブラリのサブディレクトリを追加
add_subdirectory(LibAzureKinect/KinectBodyTracker)

# Component
ament_auto_add_library(
  bt_streaming
  bt_streaming.cpp
)

# KinectBodyTrackerライブラリをリンク
target_link_libraries(bt_streaming kinect_body_tracker)

# インクルードディレクトリを追加
target_include_directories(bt_streaming PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

rclcpp_components_register_nodes(
  bt_streaming
  "bt_streaming::BodyTracking"
)

# Subscriber Component
ament_auto_add_library(
  bt_subscriber
  bt_subscriber.cpp
)

# インクルードディレクトリを追加
target_include_directories(bt_subscriber PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

rclcpp_components_register_nodes(
  bt_subscriber
  "bt_streaming::BodyTrackingSubscriber"
)

# Node
ament_auto_add_executable(
  bt_streaming_node
  bt_streaming_node.cpp
)

# KinectBodyTrackerライブラリをリンク
target_link_libraries(bt_streaming_node kinect_body_tracker)

# インクルードディレクトリを追加
target_include_directories(bt_streaming_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Subscriber Node
ament_auto_add_executable(
  bt_subscriber_node
  bt_subscriber_node.cpp
)

# インクルードディレクトリを追加
target_include_directories(bt_subscriber_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()