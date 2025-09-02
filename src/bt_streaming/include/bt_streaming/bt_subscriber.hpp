#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace bt_streaming
{
  class BodyTrackingSubscriber : public rclcpp::Node
  {
  public:
    explicit BodyTrackingSubscriber(const rclcpp::NodeOptions &options);
    virtual ~BodyTrackingSubscriber();

  private:
    // サブスクライバーのコンテナ
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_state_subscribers_;
    
    // ネームスペース一覧
    std::vector<std::string> namespaces_;
    
    // 各ネームスペースの最新データを保存
    std::map<std::string, sensor_msgs::msg::JointState::SharedPtr> latest_data_;
    
    // パラメータ
    std::string base_topic_;
    
    // 初期化関数
    void initializeSubscribers();
    
    // コールバック関数
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& namespace_name);
    
    // データ処理関数
    void processJointData(const std::string& namespace_name, const sensor_msgs::msg::JointState::SharedPtr msg);
  };
}
