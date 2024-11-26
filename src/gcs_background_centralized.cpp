#include <regex>
#include <rclcpp/rclcpp.hpp>
//#include <qos.hpp>

#include "multi_drone_slung_load_cpp/gcs_background_centralized.h"

//#include "multi_drone_slung_load_cpp/frame_transforms.h"
//#include "multi_drone_slung_load_cpp/State.h"
#include "multi_drone_slung_load_cpp/utils.h"

// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_global_position.hpp>


GCSBackgroundCentralized::GCSBackgroundCentralized() : Node("gcs_background_centralized", rclcpp::NodeOptions().use_global_arguments(true)) {
    this->ns_ = this->get_namespace();
    //this->id_ = utils::extract_id_from_name(this->ns_); //this->id_ = static_cast<int>(name_.back()) - '0'; // Assuming the last character is a digit
    
    // Get name to match with the corresponding non-pixhawk node
    // std::regex rgx("_(\\w+)_");
    // std::smatch device_type;
    // std::string name = this->get_name();

    // if (std::regex_search(name, device_type, rgx))
    // {
    //     this->device_type_ = device_type[1].str();
    //     this->name_ = this->device_type_ + std::to_string(this->id_);
    // }

    // PARAMETERS
    this->declare_parameter<int>("load_id", 1);
    this->get_parameter("load_id", this->load_id_);

    this->declare_parameter<std::string>("env", "phys");
    this->get_parameter("env", this->env_);

    // this->declare_parameter<std::string>("load_pose_type", "ground_truth");
    // this->get_parameter("load_pose_type", this->load_pose_type_);

    this->declare_parameter<float>("timer_period_gcs_background", 0.1);
    this->get_parameter("timer_period_gcs_background", this->timer_period_gcs_background_);

    // this->declare_parameter<std::string>("gt_source", "mocap");
    // this->get_parameter("gt_source", this->gt_source_);

    this->declare_parameter<int>("num_drones_", 3);
    this->get_parameter("num_drones_", this->num_drones_);

    // this->declare_parameter<std::vector<double>>("mocap_origin_lla", {42.360556, -71.093056, 10.0});
    // this->get_parameter("mocap_origin_lla", this->mocap_origin_lla_);
    
    // STATES

    // TFS
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // this->tf_static_broadcaster_init_pose_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // this->tf_static_broadcaster_world_rel_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // this->tf_static_broadcaster_item2_rel_item1_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // this->tf_static_broadcaster_item2_rel_item1_d_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // this->tf_static_broadcaster_item2_rel_item1_gt_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


    // ROS2

    //QoS settings

    rclcpp::QoS qos_profile_fmu(rclcpp::KeepLast(1));  // Equivalent to depth=1
    qos_profile_fmu.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile_fmu.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_profile_fmu.history(rclcpp::HistoryPolicy::KeepLast);
    //size_t qos_profile_fmu = 10;

    // rclcpp::QoS qos_profile_latched(rclcpp::KeepLast(1));  // Equivalent to depth=1
    // qos_profile_latched.reliability(rclcpp::ReliabilityPolicy::Reliable);
    // qos_profile_latched.durability(rclcpp::DurabilityPolicy::TransientLocal);
    // qos_profile_latched.history(rclcpp::HistoryPolicy::KeepLast);

    // rclcpp::QoS qos_profile_gz(rclcpp::KeepLast(1));  // Equivalent to depth=1
    // qos_profile_gz.reliability(rclcpp::ReliabilityPolicy::Reliable);
    // qos_profile_gz.durability(rclcpp::DurabilityPolicy::Volatile);
    // qos_profile_gz.history(rclcpp::HistoryPolicy::KeepLast);

    // rclcpp::QoS qos_profile_drone_system = rclcpp::SensorDataQoS();
    //rclcpp::QoS qos_profile_srv = rclcpp::ServicesQoS();

    // TIMERS
    int timer_period_ms = static_cast<int>(this->timer_period_gcs_background_ * 1000); 
    this->timer_cmdloop_ = this->create_wall_timer(
                                std::chrono::milliseconds(timer_period_ms),
                                std::bind(&GCSBackgroundCentralized::clbk_cmdloop, this));

    // PUBLISHERS
    // this->pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    //     this->ns_ + "/fmu/in/vehicle_command", qos_profile_fmu);

    for (int i = 1; i <= this->num_drones_; ++i) {
            // Construct the topic name dynamically based on the drone index
            std::string topic_prefix = "/px4_" + std::to_string(i);
            
            RCLCPP_INFO(this->get_logger(), topic_prefix.c_str());

            // Offboard publishers
            std::string topic_offboard = topic_prefix + "/fmu/in/offboard_control_mode";

            auto pub_offboard = this->create_publisher<px4_msgs::msg::OffboardControlMode>(topic_offboard, qos_profile_fmu);
            this->pub_offboard_modes_.push_back(pub_offboard);

            // Setpoint publishers
            std::string topic_traj = topic_prefix + "/fmu/in/trajectory_setpoint";

            auto pub_traj = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(topic_traj, qos_profile_fmu);
            this->pub_trajectories_.push_back(pub_traj);

            // Vehicle command publishers
            std::string topic_vehicle_cmd = topic_prefix + "/fmu/in/vehicle_command";

            auto pub_cmd = this->create_publisher<px4_msgs::msg::VehicleCommand>(topic_vehicle_cmd, qos_profile_fmu);
            this->pub_vehicle_commands_.push_back(pub_cmd);

    }

    // this->pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    //     "/px4_" + std::to_string(i) + "/fmu/in/vehicle_command", qos_profile_fmu);

    
    // this->pub_global_init_pose_ = this->create_publisher<multi_drone_slung_load_interfaces::msg::GlobalPose>(
    //     this->ns_ + "/out/global_init_pose", qos_profile_latched);

    // SUBSCRIBERS
    // DRONE 
    // this->sub_vehicle_phase = this->create_subscription<multi_drone_slung_load_interfaces::msg::Phase>(
    //     this->ns_ + "/out/current_phase", 
    //     qos_profile_drone_system,
    //     std::bind(&Pixhawk::clbk_change_phase, this, std::placeholders::_1)
    // );

    // FMU
    // this->sub_attitude_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    //     this->ns_ + "/fmu/out/vehicle_attitude", 
    //     qos_profile_fmu,
    //     std::bind(&Pixhawk::clbk_vehicle_attitude, this, std::placeholders::_1)
    // );

    // this->sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    //     this->ns_ + "/fmu/out/vehicle_local_position", 
    //     qos_profile_fmu,
    //     std::bind(&Pixhawk::clbk_vehicle_local_position, this, std::placeholders::_1)
    // );

    // if(this->gt_source_ != "mocap"){
    //     this->sub_global_pos_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    //         this->ns_ + "/fmu/out/vehicle_global_position", 
    //         qos_profile_fmu,
    //         std::bind(&Pixhawk::clbk_vehicle_global_position, this, std::placeholders::_1)
    //     );
    // }

    // Ground truth (this could be moved to another node if required)
    // if((this->load_pose_type_ == "ground_truth" || this->evaluate_) && (this->env_ == "sim")){
    //     std::string topic_name = "px4_";

    //     if(this->device_type_ == "load"){
    //         topic_name = "load_";
    //     }

    //     topic_name = topic_name + std::to_string(this->id_) + "/out/pose_ground_truth/gz";

    //     this->sub_pose_gt_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //         this->ns_ + "/out/pose_ground_truth/gz", 
    //         qos_profile_gz,
    //         std::bind(&Pixhawk::clbk_gt, this, std::placeholders::_1)
    //     );
    // }

    // SERVICES
    this->srv_phase_change_ = this->create_service<multi_drone_slung_load_interfaces::srv::PhaseChange>(
        "/gcs_background_" + std::to_string(this->load_id_) + "/phase_change_request", 
        std::bind(&GCSBackgroundCentralized::clbk_phase_change, this, std::placeholders::_1, std::placeholders::_2)
    );
    // qos_profile_srv

    // Print info
    RCLCPP_INFO(this->get_logger(), "CENTRALIZED GCS BACKGROUND NODE");

}

// CALLBACKS
void GCSBackgroundCentralized::clbk_cmdloop(){
    // Publish offboard mode heartbeat to all drones
    uint64_t timestamp = int(this->get_clock()->now().nanoseconds() / 1000);

    // TEMP TRAJ MESSAGE
    px4_msgs::msg::TrajectorySetpoint trajectory_msg = px4_msgs::msg::TrajectorySetpoint();
    trajectory_msg.timestamp = timestamp;
    trajectory_msg.position[0] = 1.0;
    trajectory_msg.position[1] = 0.0;
    trajectory_msg.position[2] = -3.0;

    trajectory_msg.yaw = 0.0;

    for (int i = 0; i < this->num_drones_; ++i)
    {
        // Publish heartbeat
        const auto &pub_heartbeat = this->pub_offboard_modes_[i];
        utils::publish_offboard_control_heartbeat_signal(pub_heartbeat, "pos", timestamp);

        // TODO: Publish setpoints to all drones (first test then actual)
        //const auto &pub_traj_msg = this->pub_trajectories_[i];
        this->pub_trajectories_[i]->publish(trajectory_msg);
    }

    // Set origin for all drones so a global pose estimate appears in QGC
    //int i = 1;

    // SETTING THE ORIGIN CAUSES ISSUES - just don't do it
    // for (const auto& pub_cmd : this->pub_vehicle_commands_) {
    //     utils::set_origin(pub_cmd, this->get_clock()->now(), this->mocap_origin_lla_[0], this->mocap_origin_lla_[1], this->mocap_origin_lla_[2]);
    //     RCLCPP_INFO(this->get_logger(), "Set origin drone %d", i);
    //     ++i;
    // }
}

// Take in user commands to change the phase of the drones
void GCSBackgroundCentralized::clbk_phase_change(const std::shared_ptr<multi_drone_slung_load_interfaces::srv::PhaseChange::Request> request,
         std::shared_ptr<multi_drone_slung_load_interfaces::srv::PhaseChange::Response> response)
{
    this->phase_ = request->phase_request.phase;
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Changing to phase: %d", this->phase_);
}

// HELPERS



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GCSBackgroundCentralized>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}