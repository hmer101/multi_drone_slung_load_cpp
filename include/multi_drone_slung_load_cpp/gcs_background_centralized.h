#ifndef GCS_BACKGROUND_CENTRALIZED_H
#define GCS_BACKGROUND_CENTRALIZED_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
//#include "std_msgs/msg/string.hpp"

#include "multi_drone_slung_load_cpp/State.h"

// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
#include "multi_drone_slung_load_interfaces/msg/global_pose.hpp"

#include "multi_drone_slung_load_interfaces/srv/phase_change.hpp"


class GCSBackgroundCentralized : public rclcpp::Node {
public:
    GCSBackgroundCentralized();
    //~Pixhawk();

    // HELPER FUNCTIONS
    //void set_flag_gps_home();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    //int id_; // ID of the pixhawk this node is connected to

    //std::string name_;
    std::string env_;
    // std::string device_type_;
    // std::string load_pose_type_;
    float timer_period_gcs_background_;
    // std::string gt_source_;
    int load_id_;
    int num_drones_;
    uint8_t phase_;
    // int num_cameras_;
    //std::vector<double> mocap_origin_lla_;

    // VARIABLES
    //multi_drone_slung_load_interfaces::msg::Phase current_phase_;
    //uint8_t current_phase_;
    rclcpp::TimerBase::SharedPtr timer_cmdloop_;

    // TFS
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_init_pose_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_world_rel_gt_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_d_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_gt_;
   

    // FLAGS


    // PUBLISHERS
    std::vector<rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr> pub_offboard_modes_;
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr> pub_vehicle_commands_;
    std::vector<rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr> pub_trajectories_;
    //rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
    //rclcpp::Publisher<multi_drone_slung_load_interfaces::msg::GlobalPose>::SharedPtr pub_global_init_pose_;

    // SUBSCRIBERS
    // rclcpp::Subscription<multi_drone_slung_load_interfaces::msg::Phase>::SharedPtr sub_vehicle_phase;
    // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_gt_;

    // rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
    // rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    // rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr sub_global_pos_;

    // SERVICES
    rclcpp::Service<multi_drone_slung_load_interfaces::srv::PhaseChange>::SharedPtr srv_phase_change_;

    // CALLBACKS
    void clbk_cmdloop();
    void clbk_phase_change(const std::shared_ptr<multi_drone_slung_load_interfaces::srv::PhaseChange::Request> request,
                           std::shared_ptr<multi_drone_slung_load_interfaces::srv::PhaseChange::Response> response);

    // void clbk_change_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg);
    // void clbk_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    // void clbk_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    // void clbk_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    // void clbk_gt(const geometry_msgs::msg::PoseArray msg);

    // HELPER FUNCTIONS
    // void reset();
};

#endif // GCS_BACKGROUND_CENTRALIZED_H