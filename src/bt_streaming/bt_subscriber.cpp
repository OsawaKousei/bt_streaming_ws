#include "bt_streaming/bt_subscriber.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>

namespace bt_streaming
{

  BodyTrackingSubscriber::BodyTrackingSubscriber(const rclcpp::NodeOptions &options)
      : Node("body_tracking_subscriber", options)
  {
    // パラメータの宣言と取得
    this->declare_parameter("base_topic", "joint_status");
    this->declare_parameter("namespaces", std::vector<std::string>{"body_tracking"});

    base_topic_ = this->get_parameter("base_topic").as_string();
    namespaces_ = this->get_parameter("namespaces").as_string_array();

    RCLCPP_INFO(this->get_logger(), "Initializing BodyTrackingSubscriber");
    RCLCPP_INFO(this->get_logger(), "Base topic: %s", base_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Number of namespaces: %zu", namespaces_.size());

    // トピック存在確認
    RCLCPP_INFO(this->get_logger(), "Checking topic availability...");
    bool all_topics_available = true;
    
    for (const auto& namespace_name : namespaces_)
    {
      std::string topic_name = "/" + namespace_name + "/" + base_topic_;
      
      RCLCPP_INFO(this->get_logger(), "Checking topic: %s", topic_name.c_str());
      
      if (!checkTopicExists(topic_name))
      {
        RCLCPP_ERROR(this->get_logger(), "Topic '%s' is not available", topic_name.c_str());
        all_topics_available = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Topic '%s' is available", topic_name.c_str());
      }
    }

    if (!all_topics_available)
    {
      RCLCPP_ERROR(this->get_logger(), "Some required topics are not available. Shutting down node.");
      rclcpp::shutdown();
      return;
    }

    // サブスクライバーの初期化
    initializeSubscribers();

    RCLCPP_INFO(this->get_logger(), "BodyTrackingSubscriber initialized successfully");
  }

  BodyTrackingSubscriber::~BodyTrackingSubscriber()
  {
    RCLCPP_INFO(this->get_logger(), "BodyTrackingSubscriber destroyed");
  }

  bool BodyTrackingSubscriber::checkTopicExists(const std::string& topic_name, int timeout_sec)
  {
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::seconds(timeout_sec);

    while (std::chrono::steady_clock::now() - start_time < timeout_duration)
    {
      auto topic_names_and_types = this->get_topic_names_and_types();
      
      if (topic_names_and_types.find(topic_name) != topic_names_and_types.end())
      {
        return true;
      }

      // 100ms待機してから再確認
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(this->get_node_base_interface());
    }

    return false;
  }

  void BodyTrackingSubscriber::initializeSubscribers()
  {
    for (const auto& namespace_name : namespaces_)
    {
      std::string topic_name = namespace_name + "/" + base_topic_;
      
      RCLCPP_INFO(this->get_logger(), "Creating subscriber for topic: %s", topic_name.c_str());

      auto subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
          topic_name,
          10,
          [this, namespace_name](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointStateCallback(msg, namespace_name);
          });

      joint_state_subscribers_.push_back(subscriber);
      
      // 最新データ用のマップを初期化
      latest_data_[namespace_name] = nullptr;
    }
  }

  void BodyTrackingSubscriber::jointStateCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg, 
      const std::string& namespace_name)
  {
    RCLCPP_DEBUG(this->get_logger(), 
                "Received joint state from namespace: %s with %zu joints", 
                namespace_name.c_str(), msg->name.size());

    // データ処理
    processJointData(namespace_name, msg);
  }

  void BodyTrackingSubscriber::processJointData(
      const std::string& namespace_name, 
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 最新データを更新
    latest_data_[namespace_name] = msg;

    // ここで受信したデータに対する処理を実装
    // 例：ログ出力、データ統合、別のトピックへの再パブリッシュなど
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Processing data from %s: %zu joints, timestamp: %d.%d",
                         namespace_name.c_str(),
                         msg->name.size(),
                         msg->header.stamp.sec,
                         msg->header.stamp.nanosec);

    // 例：特定の条件で全ネームスペースのデータを集約
    bool all_data_available = true;
    for (const auto& ns : namespaces_)
    {
      if (latest_data_[ns] == nullptr)
      {
        all_data_available = false;
        break;
      }
    }

    if (all_data_available)
    {
      RCLCPP_DEBUG(this->get_logger(), "All namespace data is available for processing");
      // ここで全データを使った処理を実装可能
    }
  }

} // namespace bt_streaming

// コンポーネントとして登録
RCLCPP_COMPONENTS_REGISTER_NODE(bt_streaming::BodyTrackingSubscriber)
