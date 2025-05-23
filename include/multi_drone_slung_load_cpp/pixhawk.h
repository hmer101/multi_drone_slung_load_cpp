#ifndef PIXHAWK_H
#define PIXHAWK_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
//#include "std_msgs/msg/string.hpp"

#include "multi_drone_slung_load_cpp/State.h"

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/point.hpp> 
#include <geometry_msgs/msg/quaternion.hpp>

#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
#include "multi_drone_slung_load_interfaces/msg/global_pose.hpp"
#include "multi_drone_slung_load_interfaces/msg/pixhawk_status.hpp"


class Pixhawk : public rclcpp::Node {
public:
    Pixhawk();
    //~Pixhawk();

    // HELPER FUNCTIONS
    void set_flag(bool& flag);
    void unset_flag(bool& flag);

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int id_; // ID of the pixhawk this node is connected to

    std::string name_;
    std::string env_;
    std::string frame_system_;
    std::string device_type_;
    std::string load_pose_type_;
    bool evaluate_;
    std::string gt_source_;
    int num_cameras_;
    int first_drone_num_;
    std::vector<double> mocap_origin_lla_;

    // STATES
    droneState::State global_origin_state_;
    droneState::State global_origin_state_prev_;
    droneState::State initial_global_state_;
    droneState::State initial_state_rel_world_;
    droneState::State initial_local_state_;
    droneState::State local_state_;
    droneState::State gt_state_;

    // VARIABLES
    //multi_drone_slung_load_interfaces::msg::Phase current_phase_;
    uint8_t current_phase_;

    // TIMERS
    rclcpp::TimerBase::SharedPtr timer_;

    // TFS
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_init_pose_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_world_rel_gt_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_d_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_gt_;
   

    // FLAGS
    bool flag_gps_home_set_;
    bool flag_global_origin_set_;
    bool flag_global_init_att_set_;
    bool flag_local_init_pose_set_;


    // PUBLISHERS
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
    rclcpp::Publisher<multi_drone_slung_load_interfaces::msg::GlobalPose>::SharedPtr pub_global_init_pose_;
    rclcpp::Publisher<multi_drone_slung_load_interfaces::msg::PixhawkStatus>::SharedPtr pub_pixhawk_status_;

    // SUBSCRIBERS
    rclcpp::Subscription<multi_drone_slung_load_interfaces::msg::Phase>::SharedPtr sub_vehicle_phase;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_gt_;

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;
    rclcpp::Subscription<multi_drone_slung_load_interfaces::msg::GlobalPose>::SharedPtr sub_global_origin_;

    // CALLBACKS
    void clbk_pub_pixhawk_status();
    void clbk_change_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg);
    void clbk_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void clbk_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void clbk_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void clbk_global_origin(const multi_drone_slung_load_interfaces::msg::GlobalPose msg);
    void clbk_gt(const geometry_msgs::msg::PoseArray msg);

    // HELPER FUNCTIONS
    geometry_msgs::msg::Point toMsg(const Eigen::Vector3d& vec);
    geometry_msgs::msg::Quaternion toMsg(const tf2::Quaternion& q);
    void reset();
    void publish_pixhawk_status();
};

#endif // PIXHAWK_H