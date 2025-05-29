#include <regex>
#include <rclcpp/rclcpp.hpp>
//#include <qos.hpp>

#include "multi_drone_slung_load_cpp/pixhawk.h"

#include "multi_drone_slung_load_cpp/frame_transforms.h"
//#include "multi_drone_slung_load_cpp/State.h"
#include "multi_drone_slung_load_cpp/utils.h"

// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_global_position.hpp>


Pixhawk::Pixhawk() : Node("pixhawk", rclcpp::NodeOptions().use_global_arguments(true)) {
    this->ns_ = this->get_namespace();
    this->id_ = utils::extract_id_from_name(this->ns_); //this->id_ = static_cast<int>(name_.back()) - '0'; // Assuming the last character is a digit
    
    // Get name to match with the corresponding non-pixhawk node
    std::regex rgx("_(\\w+)_");
    std::smatch device_type;
    std::string name = this->get_name();

    if (std::regex_search(name, device_type, rgx))
    {
        this->device_type_ = device_type[1].str();
        this->name_ = this->device_type_ + std::to_string(this->id_);
    }

    // PARAMETERS 
    this->declare_parameter<std::string>("env", "phys");
    this->get_parameter("env", this->env_);

    this->declare_parameter<std::string>("frame_system", "mocap");
    this->get_parameter("frame_system", this->frame_system_);

    this->declare_parameter<std::string>("load_pose_type", "ground_truth");
    this->get_parameter("load_pose_type", this->load_pose_type_);

    this->declare_parameter<bool>("evaluate", false);
    this->get_parameter("evaluate", this->evaluate_);

    this->declare_parameter<std::string>("gt_source", "mocap");
    this->get_parameter("gt_source", this->gt_source_);

    this->declare_parameter<int>("num_cameras", 0);
    this->get_parameter("num_cameras", this->num_cameras_);

    this->declare_parameter<int>("first_drone_num", 1);
    this->get_parameter("first_drone_num", this->first_drone_num_);

    this->declare_parameter<std::vector<double>>("mocap_origin_lla", {42.360556, -71.093056, 10.0});
    this->get_parameter("mocap_origin_lla", this->mocap_origin_lla_);
    
    // STATES
    this->global_origin_state_ = droneState::State("globe", droneState::CS_type::LLA);
    this->global_origin_state_prev_ = this->global_origin_state_.copy();

    this->initial_global_state_ = droneState::State("globe", droneState::CS_type::LLA);
    this->initial_state_rel_world_ = droneState::State("local_ref", droneState::CS_type::ENU);
    this->initial_local_state_ = droneState::State(this->name_ + "_init", droneState::CS_type::ENU);
    this->local_state_ = droneState::State(this->name_ + "_init", droneState::CS_type::ENU);

    this->gt_state_ = droneState::State("ground_truth", droneState::CS_type::XYZ);

    // TIMER
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Pixhawk::clbk_pub_pixhawk_status, this)
    );

    // TFS
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->tf_static_broadcaster_init_pose_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_world_rel_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_d_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->tf_static_broadcaster_item2_rel_item1_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // FLAGS
    this->flag_gps_home_set_ = false; // GPS home set when vehicle armed
    this->flag_global_origin_set_ = false; // Global origin set when the first drone publishes its global pose
    this->flag_global_init_att_set_ = false;
    this->flag_local_init_pose_set_ = false;

    //this->reset();


    // ROS2

    //QoS settings
    //rclcpp::QoS qos_profile_cam = rclcpp::SensorDataQoS();

    // rclcpp::SubscriptionOptions sub_opt_fmu;
    // options.qos_profile = rclcpp::QoS(
    //     rclcpp::KeepLast(10),
    //     rclcpp::ReliabilityPolicy::BestEffort,
    //     rclcpp::DurabilityPolicy::Volatile,
    //     rclcpp::HistoryPolicy::KeepLast,
    //     rclcpp::Deadline(std::chrono::milliseconds(500)),
    //     rclcpp::Lifespan(std::chrono::seconds(10)),
    //     rclcpp::LivelinessPolicy::Automatic,
    //     rclcpp::LivelinessLeaseDuration(std::chrono::seconds(2))
    // );

    rclcpp::QoS qos_profile_fmu(rclcpp::KeepLast(1));  // Equivalent to depth=1
    qos_profile_fmu.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile_fmu.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_profile_fmu.history(rclcpp::HistoryPolicy::KeepLast);
    //size_t qos_profile_fmu = 10;

    // !!!! This is now set to qos_profile_sensor_data settings !!!!!
    rclcpp::QoS qos_profile_latched(rclcpp::KeepLast(1));  // Equivalent to depth=1
    qos_profile_latched.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile_latched.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile_latched.history(rclcpp::HistoryPolicy::KeepLast);

    // rclcpp::QoS qos_profile_latched(rclcpp::KeepLast(1));  // Equivalent to depth=1
    // qos_profile_latched.reliability(rclcpp::ReliabilityPolicy::Reliable);
    // qos_profile_latched.durability(rclcpp::DurabilityPolicy::TransientLocal);
    // qos_profile_latched.history(rclcpp::HistoryPolicy::KeepLast);

    rclcpp::QoS qos_profile_gz(rclcpp::KeepLast(1));  // Equivalent to depth=1
    qos_profile_gz.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile_gz.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile_gz.history(rclcpp::HistoryPolicy::KeepLast);

    rclcpp::QoS qos_profile_drone_system = rclcpp::SensorDataQoS(); 
    

    // PUBLISHERS
    this->pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        this->ns_ + "/fmu/in/vehicle_command", qos_profile_fmu);

    this->pub_global_init_pose_ = this->create_publisher<multi_drone_slung_load_interfaces::msg::GlobalPose>(
        this->ns_ + "/out/global_init_pose", qos_profile_latched);

    this->pub_pixhawk_status_ = this->create_publisher<multi_drone_slung_load_interfaces::msg::PixhawkStatus>(
        this->ns_ + "/out/pixhawk_status", qos_profile_latched);

    // SUBSCRIBERS
    // DRONE 
    this->sub_vehicle_phase = this->create_subscription<multi_drone_slung_load_interfaces::msg::Phase>(
        this->ns_ + "/out/current_phase", 
        qos_profile_drone_system,
        std::bind(&Pixhawk::clbk_change_phase, this, std::placeholders::_1)
    );

    // FMU
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

    //if(this->gt_source_ != "mocap"){
    this->sub_global_pos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        this->ns_ + "/fmu/out/vehicle_global_position", 
        qos_profile_fmu,
        std::bind(&Pixhawk::clbk_vehicle_global_position, this, std::placeholders::_1)
    );
    //}

    // Ground truth (this could be moved to another node if required)
    if((this->load_pose_type_ == "ground_truth" || this->evaluate_) && (this->env_ == "sim")){
        std::string topic_name = "px4_";

        if(this->device_type_ == "load"){
            topic_name = "load_";
        }

        topic_name = topic_name + std::to_string(this->id_) + "/out/pose_ground_truth/gz";

        this->sub_pose_gt_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            this->ns_ + "/out/pose_ground_truth/gz", 
            qos_profile_gz,
            std::bind(&Pixhawk::clbk_gt, this, std::placeholders::_1)
        );
    }

    // If non-mocap, subscribe to the first drone to get the global origin (if this is not it)
    if(this->gt_source_ != "mocap" && ((this->device_type_ == "drone" && this->id_ != this->first_drone_num_) || (this->device_type_ == "load"))){    
        this->sub_global_origin_ = this->create_subscription<multi_drone_slung_load_interfaces::msg::GlobalPose>(
            "/px4_" + std::to_string(this->first_drone_num_) + "/out/global_init_pose", 
            qos_profile_latched,
            std::bind(&Pixhawk::clbk_global_origin, this, std::placeholders::_1)
        );
    }

    // Print info
    RCLCPP_INFO(this->get_logger(), "PIXHAWK NODE %d", this->id_);

}

// CALLBACKS
void Pixhawk::clbk_pub_pixhawk_status() {
    this->publish_pixhawk_status();
}

void Pixhawk::clbk_change_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg) {
    this->current_phase_ = msg->phase;
}

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
        //this->set_flag_global_init_att();
        this->set_flag(this->flag_global_init_att_set_);

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
        // else if(this->frame_system_ == "mocap" && !this->flag_gps_home_set_){ //Set the mocap origin if it hasn't been set yet. TODO: Ensure this doesn't cause problems for not centralized version
        //     // Set the GPS home for the mocap system
        //     utils::set_origin(this->pub_vehicle_command_, this->get_clock()->now(), this->mocap_origin_lla_[0], this->mocap_origin_lla_[1], this->mocap_origin_lla_[2]);
        // }
    }
}

void Pixhawk::clbk_vehicle_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
    // Check if GPS home is not set and phase is true
    bool correct_phase = (this->device_type_ == "drone" && this->current_phase_ == multi_drone_slung_load_interfaces::msg::Phase::PHASE_SETUP_DRONE) 
        || (this->device_type_ == "load" && this->current_phase_ == multi_drone_slung_load_interfaces::msg::Phase::PHASE_SETUP_LOAD); //TODO: TEST THIS ACTUALLY WORKS ON LOAD (might have to use same logic as load and check if all drones are in load setup phase)

    if (correct_phase) { //!this->flag_gps_home_set_ && TODO: MAKE SURE IT WORKS WITHOUT THIS!!
        // Set the initial global position (lat, lon, alt)
        this->initial_global_state_.setPos(Eigen::Vector3d(msg->lat, msg->lon, msg->alt));

        // Reset origin if frame system is not mocap
        if(this->gt_source_ != "mocap"){
            utils::set_origin(this->pub_vehicle_command_, this->get_clock()->now(), msg->lat, msg->lon, msg->alt);

            // Create the message to publish
            multi_drone_slung_load_interfaces::msg::GlobalPose msg_global_pose;
            msg_global_pose.global_pos.lat = this->initial_global_state_.getPos()[0];
            msg_global_pose.global_pos.lon = this->initial_global_state_.getPos()[1];
            msg_global_pose.global_pos.alt = this->initial_global_state_.getPos()[2];

            // Set the global attitude quaternion (converted from internal state)
            msg_global_pose.global_att.q[0] = this->initial_global_state_.getAtt().w();
            msg_global_pose.global_att.q[1] = this->initial_global_state_.getAtt().x();
            msg_global_pose.global_att.q[2] = this->initial_global_state_.getAtt().y();
            msg_global_pose.global_att.q[3] = this->initial_global_state_.getAtt().z();

            // Publish the global pose
            this->pub_global_init_pose_->publish(msg_global_pose);
        }

        // Set the flag to indicate that GPS home has been set
        //this->set_flag_gps_home();
        this->set_flag(this->flag_gps_home_set_);
    }
}

void Pixhawk::clbk_global_origin(const multi_drone_slung_load_interfaces::msg::GlobalPose msg) {
    this->global_origin_state_.setPos(Eigen::Vector3d(msg.global_pos.lat, msg.global_pos.lon, msg.global_pos.alt));

    //TODO: JUST UPDATED THISSSSSSS HEREEEEEE
    // tf2::Quaternion is x,y,z,w; PX4 global attitude is w,x,y,z
    this->global_origin_state_.setAtt(tf2::Quaternion(msg.global_att.q[1], msg.global_att.q[2], msg.global_att.q[3], msg.global_att.q[0])); //msg.global_att.q[0], msg.global_att.q[1], msg.global_att.q[2], msg.global_att.q[3]));

    RCLCPP_INFO(this->get_logger(), "UPDATED GLOBAL ORIGIN POS: %f %f %f", this->global_origin_state_.getPos()[0], this->global_origin_state_.getPos()[1], this->global_origin_state_.getPos()[2]);
    RCLCPP_INFO(this->get_logger(), "UPDATED GLOBAL ORIGIN ATT (x,y,z,w): %f %f %f %f", this->global_origin_state_.getAtt()[0], this->global_origin_state_.getAtt()[1], this->global_origin_state_.getAtt()[2], this->global_origin_state_.getAtt()[3]);

    // Global origin updated - must update local initial poses
    this->set_flag(this->flag_global_origin_set_);
    this->unset_flag(this->flag_local_init_pose_set_);
}


void Pixhawk::clbk_gt(const geometry_msgs::msg::PoseArray msg) {
    // Ground truth pose index changes depending on the device and the number of cameras
    size_t pose_ind = 2; // For drones in simulation

    if(this->device_type_ == "load" || this->num_cameras_ == 0){ // For load and when no cameras are used
        pose_ind = 1;
    }

    // Update the ground truth pose (broadcast and store)
    this->gt_state_ = utils::update_ground_truth_pose(msg, this->get_clock()->now(), this->name_, *(this->tf_broadcaster_), pose_ind = pose_ind);
}


// HELPER FUNCTIONS
geometry_msgs::msg::Point Pixhawk::toMsg(const Eigen::Vector3d& vec) {
    geometry_msgs::msg::Point msg;
    msg.x = vec.x();
    msg.y = vec.y();
    msg.z = vec.z();

    return msg;
}

geometry_msgs::msg::Quaternion Pixhawk::toMsg(const tf2::Quaternion& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();

    return msg;
}

void Pixhawk::set_flag(bool& flag) {
    flag = true;
    this->publish_pixhawk_status();
}

void Pixhawk::unset_flag(bool& flag) {
    flag = false;
    this->publish_pixhawk_status();
}

void Pixhawk::reset(){
    this->flag_gps_home_set_ = false;
    this->flag_local_init_pose_set_ = false;
    this->flag_global_init_att_set_ = false;

    // Publish the status (note cannot be done before the node is initialized i.e. cannot run this in the constructor)
    this->publish_pixhawk_status();
}

void Pixhawk::publish_pixhawk_status() {
    // Flags
    multi_drone_slung_load_interfaces::msg::PixhawkStatus msg_pixhawk_status;
    msg_pixhawk_status.gps_home_set = this->flag_gps_home_set_;
    msg_pixhawk_status.global_origin_set = this->flag_global_origin_set_;
    msg_pixhawk_status.local_init_pose_set = this->flag_local_init_pose_set_;
    msg_pixhawk_status.global_init_att_set = this->flag_global_init_att_set_;

    // Initial global state
    msg_pixhawk_status.initial_global_position = toMsg(this->initial_global_state_.getPos());
    msg_pixhawk_status.initial_global_attitude = toMsg(this->initial_global_state_.getAtt());

    // Global origin 
    msg_pixhawk_status.global_origin_position = toMsg(this->global_origin_state_.getPos());
    msg_pixhawk_status.global_origin_attitude = toMsg(this->global_origin_state_.getAtt());

    // Publish the status
    this->pub_pixhawk_status_->publish(msg_pixhawk_status);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pixhawk>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}