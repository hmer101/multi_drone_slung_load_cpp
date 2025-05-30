#include "multi_drone_slung_load_cpp/tf_static_to_tf_relay.h"

TfStaticToTfRelay::TfStaticToTfRelay()
: Node("tf_static_to_tf_relay")
{
  // QoS for /tf_static
  rclcpp::QoS qos_sub(10);
  qos_sub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos_sub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  // QoS for /tf
  rclcpp::QoS qos_pub(10);
  qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos_pub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", qos_sub,
    std::bind(&TfStaticToTfRelay::tfStaticCallback, this, std::placeholders::_1));

  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos_pub);

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&TfStaticToTfRelay::publishTfTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "TF Static to TF Relay initialized");
}

void TfStaticToTfRelay::tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  for (const auto &transform : msg->transforms)
  {
    // Remove any existing transform with same parent + child
    auto it = std::remove_if(
      stored_transforms_.begin(), stored_transforms_.end(),
      [&](const geometry_msgs::msg::TransformStamped &t) {
        return t.header.frame_id == transform.header.frame_id &&
               t.child_frame_id == transform.child_frame_id;
      });

    stored_transforms_.erase(it, stored_transforms_.end());

    // Add the new transform (with updated timestamp)
    stored_transforms_.push_back(transform);
  }

  //RCLCPP_INFO(this->get_logger(), "Subed");
}

void TfStaticToTfRelay::publishTfTimerCallback()
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf2_msgs::msg::TFMessage msg;
  rclcpp::Time now = this->now();

  for (auto &transform : stored_transforms_)
  {
    transform.header.stamp = now;
    msg.transforms.push_back(transform);
  }

  tf_pub_->publish(msg);

  //RCLCPP_INFO(this->get_logger(), "Repubed");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfStaticToTfRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}