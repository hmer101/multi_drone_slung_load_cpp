#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <vector>
#include <mutex>

class TfStaticToTfRelay : public rclcpp::Node
{
public:
  TfStaticToTfRelay();

private:
  void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void publishTfTimerCallback();

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::vector<geometry_msgs::msg::TransformStamped> stored_transforms_;
  std::mutex tf_mutex_;  // Thread safety for concurrent access
};
