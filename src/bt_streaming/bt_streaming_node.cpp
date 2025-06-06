#include "rclcpp/rclcpp.hpp"
#include "bt_streaming/bt_streaming.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto bt_streaming_node = std::make_shared<bt_streaming::BodyTracking>(rclcpp::NodeOptions());
  exec.add_node(bt_streaming_node);

  exec.spin();
  rclcpp::shutdown();
}