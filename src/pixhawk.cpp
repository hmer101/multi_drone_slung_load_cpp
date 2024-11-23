#include <regex>
#include <rclcpp/rclcpp.hpp>

#include "multi_drone_slung_load_cpp/pixhawk.h"

#include "multi_drone_slung_load_cpp/frame_transforms.h"
//#include "multi_drone_slung_load_cpp/State.h"
#include "multi_drone_slung_load_cpp/utils.h"

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


Pixhawk::Pixhawk() : Node("pixhawk", rclcpp::NodeOptions().use_global_arguments(true)) {
    this->ns_ = this->get_namespace();
    this->id_ = utils::extract_id_from_name(this->ns_); //this->id_ = static_cast<int>(name_.back()) - '0'; // Assuming the last character is a digit
    
    // Get name to match with the corresponding non-pixhawk node
    std::regex rgx("_(\\w+)_");
    std::smatch device_type;
    std::string name = this->get_name();

    if (std::regex_search(name, device_type, rgx))
    {
        this->name_ = device_type[1].str() + std::to_string(this->id_);
    }

    // PARAMETERS
    this->declare_parameter<std::string>("env", "phys");
    this->get_parameter("env", this->env_);

    this->declare_parameter<std::string>("load_pose_type", "ground_truth");
    this->get_parameter("load_pose_type", this->load_pose_type_);

    this->declare_parameter<bool>("evaluate", false);
    this->get_parameter("evaluate", this->evaluate_);

    this->declare_parameter<std::string>("gt_source", "mocap");
    this->get_parameter("gt_source", this->gt_source_);
    
    // STATES
    this->global_origin_state_ = droneState::State("globe", droneState::CS_type::LLA);
    this->global_origin_state_prev_ = this->global_origin_state_.copy();

    this->initial_global_state_ = droneState::State("globe", droneState::CS_type::LLA);
    this->initial_state_rel_world_ = droneState::State("local_ref", droneState::CS_type::ENU);
    this->initial_local_state_ = droneState::State(this->name_ + "_init", droneState::CS_type::ENU);
    this->local_state_ = droneState::State(this->name_ + "_init", droneState::CS_type::ENU);

    this->gt_state_ = droneState::State("ground_truth", droneState::CS_type::XYZ);

    // TFS
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->tf_static_broadcaster_init_pose_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_world_rel_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_d_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // FLAGS
    this->flag_gps_home_set_ = false; // GPS home set when vehicle armed
    this->flag_global_init_att_set_ = false;
    this->flag_local_init_pose_set_ = false;


    // ROS2

    //QoS settings
    //rclcpp::QoS qos_profile_cam = rclcpp::SensorDataQoS();

    rclcpp::QoS qos_profile_fmu(rclcpp::KeepLast(1));  // Equivalent to depth=1
    qos_profile_fmu.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile_fmu.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_profile_fmu.history(rclcpp::HistoryPolicy::KeepLast);


    // SUBSCRIBERS
    this->sub_attitude_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        this->ns_ + "/fmu/out/vehicle_attitude", 
        qos_profile_fmu,
        std::bind(&Pixhawk::clbk_vehicle_attitude, this, std::placeholders::_1)
    );

    this->sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->ns_ + "/fmu/out/vehicle_local_position", 
        qos_profile_fmu,
        std::bind(&Pixhawk::clbk_vehicle_local_position, this, std::placeholders::_1)
    );


    // Print info
    RCLCPP_INFO(this->get_logger(), "PIXHAWK NODE %d", this->id_);

}

// CALLBACKS
void Pixhawk::clbk_vehicle_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    // Convert quaternion from PX4 (FRD->NED) to ROS (FLU->ENU)
    Eigen::Quaterniond q_px4(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Quaterniond q_ros = frame_transforms::px4_to_ros_orientation(q_px4); 

    // Update the local state with the new orientation
    this->local_state_.setAtt(utils::convert_quaternion_eigen_to_tf(q_ros));

    // Set the global attitude if it hasn't been set before or if the GPS is still waiting to be set
    if (!this->flag_gps_home_set_ || !this->flag_global_init_att_set_) {
        // Set the initial attitude as the current attitude
        this->initial_global_state_.setAtt(this->local_state_.getAtt());
        this->flag_global_init_att_set_ = true;

        // Set initial local state for mocap
        this->initial_local_state_.setPos(this->local_state_.getPos());
        this->initial_local_state_.setAtt(this->local_state_.getAtt());
    }
}

void Pixhawk::clbk_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // Handle NED->ENU transformation
    this->local_state_.setPos(Eigen::Vector3d(msg->y, msg->x, -msg->z));  // Position: (y, x, -z)
    this->local_state_.setVel(Eigen::Vector3d(msg->vy, msg->vx, -msg->vz));  // Velocity: (vy, vx, -vz)

    // Publish TF if orientation is valid (non-NaN)
    if (!std::isnan(this->local_state_.getAtt().x())) {
        // Broadcast TF relative to the local reference
        utils::broadcast_tf(this->get_clock()->now(), this->name_ + "_init", this->name_, this->local_state_.getPos(), utils::convert_quaternion_tf_to_eigen(this->local_state_.getAtt()), *this->tf_broadcaster_);

        // Check if we are in 'phys' environment and using GNSS for ground truth
        if (this->env_ == "phys" && this->gt_source_ == "gnss" &&
            (this->load_pose_type_ == "ground_truth" || this->evaluate_)) {
            
            // Broadcast ground truth pose TF
            utils::broadcast_tf(this->get_clock()->now(), this->name_ + "_init", this->name_ + "_gt", 
                                 this->local_state_.getPos(), utils::convert_quaternion_tf_to_eigen(this->local_state_.getAtt()), *this->tf_broadcaster_);
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pixhawk>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}