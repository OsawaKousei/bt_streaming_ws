#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <string>

#include "LibAzureKinect/KinectBodyTracker/KinectBodyTracker.h"

namespace bt_streaming
{
  class BodyTracking : public rclcpp::Node
  {
  public:
    explicit BodyTracking(const rclcpp::NodeOptions &options);
    virtual ~BodyTracking();

  private:
    // KinectBodyTrackerのインスタンス
    std::unique_ptr<kinect_tracker::KinectBodyTracker> kinect_tracker_;

    // ROS2パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // パラメータ
    double publish_rate_;
    std::string frame_id_;
    bool use_async_mode_;

    // 初期化関数
    bool initializeKinect();

    // コールバック関数
    void timerCallback();
    void bodyTrackingCallback(const kinect_tracker::BodyTrackingResult &result);

    // メッセージ変換関数
    sensor_msgs::msg::JointState convertToJointState(const kinect_tracker::BodyTrackingResult &result);
    std::string getJointName(k4abt_joint_id_t joint_id);
  };
}
