#ifndef PIXHAWK_H
#define PIXHAWK_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <tf2_ros/transform_listener.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include "std_msgs/msg/string.hpp"

//#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
#include "multi_drone_slung_load_cpp/State.h"
// #include "multi_drone_slung_load_cpp/utils.h"

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


class Pixhawk : public rclcpp::Node {
public:
    Pixhawk();
    //~Pixhawk();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int id_; // ID of the pixhawk this node is connected to

    std::string name_;
    std::string env_;
    std::string load_pose_type_;
    bool evaluate_;
    std::string gt_source_;

    // STATES
    droneState::State global_origin_state_;
    droneState::State global_origin_state_prev_;
    droneState::State initial_global_state_;
    droneState::State initial_state_rel_world_;
    droneState::State initial_local_state_;
    droneState::State local_state_;
    droneState::State gt_state_;

    // TFS
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_init_pose_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_world_rel_gt_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_d_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_item2_rel_item1_gt_;
   

    // FLAGS
    bool flag_gps_home_set_;
    bool flag_global_init_att_set_;
    bool flag_local_init_pose_set_;


    // SUBSCRIBERS
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;

    // CALLBACKS
    void clbk_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void clbk_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
};

#endif // PIXHAWK_H