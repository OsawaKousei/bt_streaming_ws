#include "bt_streaming/bt_streaming.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace bt_streaming
{

  BodyTracking::BodyTracking(const rclcpp::NodeOptions &options)
      : Node("body_tracking", options)
  {
    // パラメータの宣言と取得
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("frame_id", "kinect_link");
    this->declare_parameter("use_async_mode", true);

    publish_rate_ = this->get_parameter("publish_rate").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    use_async_mode_ = this->get_parameter("use_async_mode").as_bool();

    // パブリッシャーの作成
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "body_tracking/joint_states", 10);

    // Kinectの初期化
    if (!initializeKinect())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Kinect");
      return;
    }

    if (use_async_mode_)
    {
      // 非同期モード：コールバック関数を設定して開始
      kinect_tracker_->setBodyTrackingCallback(
          std::bind(&BodyTracking::bodyTrackingCallback, this, std::placeholders::_1));

      if (!kinect_tracker_->startTracking())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start tracking");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Body tracking started in async mode");
    }
    else
    {
      // 同期モード：タイマーで定期的に取得
      auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
      timer_ = this->create_wall_timer(timer_period,
                                       std::bind(&BodyTracking::timerCallback, this));

      RCLCPP_INFO(this->get_logger(), "Body tracking started in sync mode");
    }

    RCLCPP_INFO(this->get_logger(), "BodyTrackingNode initialized successfully");
  }

  BodyTracking::~BodyTracking()
  {
    if (kinect_tracker_)
    {
      kinect_tracker_->shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "BodyTrackingNode destroyed");
  }

  bool BodyTracking::initializeKinect()
  {
    kinect_tracker_ = std::make_unique<kinect_tracker::KinectBodyTracker>();

    kinect_tracker::KinectBodyTracker::DeviceConfig config;
    config.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.colorResolution = K4A_COLOR_RESOLUTION_OFF;
    config.fps = K4A_FRAMES_PER_SECOND_30;
    config.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;

    return kinect_tracker_->initialize(config);
  }

  void BodyTracking::timerCallback()
  {
    kinect_tracker::BodyTrackingResult result;

    if (kinect_tracker_->getBodyTrackingResult(result))
    {
      auto joint_state_msg = convertToJointState(result);
      joint_state_pub_->publish(joint_state_msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Failed to get body tracking result");
    }
  }

  void BodyTracking::bodyTrackingCallback(const kinect_tracker::BodyTrackingResult &result)
  {
    auto joint_state_msg = convertToJointState(result);
    joint_state_pub_->publish(joint_state_msg);
  }

  sensor_msgs::msg::JointState BodyTracking::convertToJointState(
      const kinect_tracker::BodyTrackingResult &result)
  {
    sensor_msgs::msg::JointState joint_state_msg;

    // ヘッダー情報の設定
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = frame_id_;

    // 各ボディの骨格情報を処理
    for (const auto &body : result.bodies)
    {
      for (const auto &joint : body.joints)
      {
        // ジョイント名の生成（ボディIDを含む）
        std::string joint_name = "body_" + std::to_string(body.id) + "_" + getJointName(joint.jointId);

        joint_state_msg.name.push_back(joint_name);

        // 位置情報の設定（メートル単位に変換）
        joint_state_msg.position.push_back(joint.position[0] / 1000.0); // x
        joint_state_msg.position.push_back(joint.position[1] / 1000.0); // y
        joint_state_msg.position.push_back(joint.position[2] / 1000.0); // z

        // 速度情報（使用しないので空）
        joint_state_msg.velocity.push_back(0.0);
        joint_state_msg.velocity.push_back(0.0);
        joint_state_msg.velocity.push_back(0.0);

        // エフォート情報として信頼度を格納
        double confidence = static_cast<double>(joint.confidenceLevel);
        joint_state_msg.effort.push_back(confidence);
        joint_state_msg.effort.push_back(confidence);
        joint_state_msg.effort.push_back(confidence);
      }
    }

    return joint_state_msg;
  }

  std::string BodyTracking::getJointName(k4abt_joint_id_t joint_id)
  {
    static const std::vector<std::string> joint_names = {
        "pelvis", "spine_navel", "spine_chest", "neck", "clavicle_left",
        "shoulder_left", "elbow_left", "wrist_left", "hand_left", "handtip_left",
        "thumb_left", "clavicle_right", "shoulder_right", "elbow_right", "wrist_right",
        "hand_right", "handtip_right", "thumb_right", "hip_left", "knee_left",
        "ankle_left", "foot_left", "hip_right", "knee_right", "ankle_right",
        "foot_right", "head", "nose", "eye_left", "ear_left",
        "eye_right", "ear_right"};

    if (joint_id >= 0 && joint_id < joint_names.size())
    {
      return joint_names[joint_id];
    }

    return "unknown_" + std::to_string(joint_id);
  }

} // namespace bt_streaming

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(bt_streaming::BodyTracking)
