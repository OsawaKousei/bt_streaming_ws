#include "rclcpp/rclcpp.hpp"
#include "bt_streaming/bt_subscriber.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto bt_subscriber_node = std::make_shared<bt_streaming::BodyTrackingSubscriber>(rclcpp::NodeOptions());
  exec.add_node(bt_subscriber_node);

  exec.spin();
  rclcpp::shutdown();
}
