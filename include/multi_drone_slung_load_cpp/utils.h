#ifndef UTILS_H
#define UTILS_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <string>
//#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include "std_msgs/msg/string.hpp"

#include "multi_drone_slung_load_cpp/State.h"
//#include "multi_drone_slung_load_cpp/pixhawk.h"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>


namespace utils {
    // TRANSFORMS
    void broadcast_tf(const rclcpp::Time &time, const std::string &frame_parent, const std::string &frame_child, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, tf2_ros::TransformBroadcaster &broadcaster);
    void broadcast_tf(const rclcpp::Time &time, const std::string &frame_parent, const std::string &frame_child, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, tf2_ros::StaticTransformBroadcaster &broadcaster);
    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf(const std::string &target_frame, const std::string &source_frame, tf2_ros::Buffer &tfBuffer, const rclcpp::Time &time, rclcpp::Logger logger);

    std::shared_ptr<droneState::State> transform_frames(const droneState::State &state, const std::string &frame2_name, tf2_ros::Buffer &tf_buffer, rclcpp::Logger logger, droneState::CS_type cs_out_type = droneState::CS_type::XYZ);
    geometry_msgs::msg::WrenchStamped transform_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr& msg, const std::string& target_frame, const std::string& source_frame, tf2_ros::Buffer& tf_buffer, const rclcpp::Time& time, rclcpp::Logger logger);

    droneState::State update_ground_truth_pose(const geometry_msgs::msg::PoseArray &gt_msg, const rclcpp::Time &time, const std::string &name_frame_child, tf2_ros::TransformBroadcaster &tf_broadcaster, size_t pose_ind = 0); //, Pixhawk *pixhawk_pose = nullptr);

    // MATH
    float getTrace(const tf2::Matrix3x3 &matrix);

    // STRING HANDLING
    int extract_id_from_name(const std::string &input);
    std::vector<float> splitAndConvert(const std::string &s, char delimiter);

    // OTHER HANDLING
    geometry_msgs::msg::Pose extract_pose_from_pose_array_msg(const geometry_msgs::msg::PoseArray &pose_array, size_t index);

    // CONVERSIONS
    Eigen::Vector3d convert_vec_floats_to_eigen(const std::vector<float> &float_vector);  
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const droneState::State &state);
    droneState::State convert_tf_stamped_msg_to_state(const geometry_msgs::msg::TransformStamped &pose_msg, std::string frame, droneState::CS_type cs_type, Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0));
    //tf2::Quaternion convert_rvec_to_quaternion(const cv::Vec3d &rvec);
    Eigen::Matrix3d convert_rvec_to_rotmat(const Eigen::Vector3d &rvec);
    tf2::Quaternion convert_quaternion_eigen_to_tf(const Eigen::Quaterniond &q);
    Eigen::Quaterniond convert_quaternion_tf_to_eigen(const tf2::Quaternion &q);

    // PX4
    int extract_instance_from_connection(const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr &pub_vehicle_command);
    void publish_vehicle_command(uint16_t command, const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr &pub_vehicle_command,
                                 const rclcpp::Time &timestamp, double param1 = 0.0, double param2 = 0.0, double param3 = 0.0, double param4 = 0.0, double param5 = 0.0, double param6 = 0.0, double param7 = 0.0);
    void set_origin(const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& pub_vehicle_command, const rclcpp::Time &timestamp, double lat, double lon, double alt);

    void publish_offboard_control_heartbeat_signal(rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode, const std::string &what_control, uint64_t timestamp);

}

#endif // UTILS_H
