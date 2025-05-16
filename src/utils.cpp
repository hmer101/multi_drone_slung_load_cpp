#include "multi_drone_slung_load_cpp/utils.h"
#include <fstream>
#include <iostream>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>

namespace utils {
    // TRANSFORMS
    void broadcast_tf(
        const rclcpp::Time &time, 
        const std::string &frame_parent, 
        const std::string &frame_child, 
        const Eigen::Vector3d &pos, 
        const Eigen::Quaterniond &att, 
        tf2_ros::TransformBroadcaster &broadcaster)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = time;
        t.header.frame_id = frame_parent;
        t.child_frame_id = frame_child;

        t.transform.translation.x = pos[0];
        t.transform.translation.y = pos[1];
        t.transform.translation.z = pos[2];

        t.transform.rotation.x = att.x();
        t.transform.rotation.y = att.y();
        t.transform.rotation.z = att.z();
        t.transform.rotation.w = att.w();

        broadcaster.sendTransform(t);
    }
    
    void broadcast_tf(
        const rclcpp::Time &time, 
        const std::string &frame_parent, 
        const std::string &frame_child, 
        const Eigen::Vector3d &pos, 
        const Eigen::Quaterniond &att, 
        tf2_ros::StaticTransformBroadcaster &broadcaster)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = time;
        t.header.frame_id = frame_parent;
        t.child_frame_id = frame_child;

        t.transform.translation.x = pos[0];
        t.transform.translation.y = pos[1];
        t.transform.translation.z = pos[2];

        t.transform.rotation.x = att.x();
        t.transform.rotation.y = att.y();
        t.transform.rotation.z = att.z();
        t.transform.rotation.w = att.w();

        broadcaster.sendTransform(t);
    }
    
    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf( 
        const std::string& target_frame, 
        const std::string& source_frame,
        tf2_ros::Buffer& tfBuffer,
        const rclcpp::Time& time, 
        rclcpp::Logger logger) 
    {
        try {
            auto transform = tfBuffer.lookupTransform(target_frame, source_frame, time);
            return transform;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger, "Failed to find transform: %s", ex.what());
            return std::nullopt;
        }
    }


    Eigen::Vector3d transform_position(const Eigen::Vector3d& p_BA, const Eigen::Vector3d& p_CB, const tf2::Quaternion& q_CB) {
        tf2::Quaternion p_BA_quat(0, p_BA.x(), p_BA.y(), p_BA.z());
        tf2::Quaternion p_BA_rotated = q_CB * p_BA_quat * q_CB.inverse();
        Eigen::Vector3d p_CA = Eigen::Vector3d(p_BA_rotated.x(), p_BA_rotated.y(), p_BA_rotated.z()) + p_CB;
        return p_CA;
    }

    tf2::Quaternion transform_orientation(const tf2::Quaternion& q_BA, const tf2::Quaternion& q_CB) {
        tf2::Quaternion q_CA = q_CB * q_BA;
        return q_CA;
    }

    std::shared_ptr<droneState::State> transform_frames(const droneState::State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger, droneState::CS_type cs_out_type) {
        std::shared_ptr<droneState::State> state2 = std::make_shared<droneState::State>(frame2_name, cs_out_type); //CS_type::ENU

        // Find the transform
        geometry_msgs::msg::TransformStamped tf_f1_rel_f2;
        try {
            tf_f1_rel_f2 = tf_buffer.lookupTransform(frame2_name, state.getFrame(), tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(logger, "Failed to find transform: %s", ex.what());
            return nullptr;
        }

        // Collect transformation vector and quaternion
        Eigen::Vector3d p_f2f1(tf_f1_rel_f2.transform.translation.x,
                            tf_f1_rel_f2.transform.translation.y,
                            tf_f1_rel_f2.transform.translation.z);

        tf2::Quaternion q_f2f1(tf_f1_rel_f2.transform.rotation.x,
                            tf_f1_rel_f2.transform.rotation.y,
                            tf_f1_rel_f2.transform.rotation.z,
                            tf_f1_rel_f2.transform.rotation.w);

        // Perform transform
        state2->setPos(transform_position(state.getPos(), p_f2f1, q_f2f1));
        state2->setAtt(transform_orientation(state.getAtt(), q_f2f1));

        return state2;
    }
    
    droneState::State update_ground_truth_pose(
        const geometry_msgs::msg::PoseArray &gt_msg,
        const rclcpp::Time &time,
        const std::string &name_frame_child,
        tf2_ros::TransformBroadcaster &tf_broadcaster,
        size_t pose_ind//, 
        //Pixhawk *pixhawk_pose
    ) {
        droneState::State state_obj_gt("ground_truth", droneState::CS_type::XYZ);
        geometry_msgs::msg::Pose drone_pose_gt;

        // If pose_ind is not -1, extract the pose from the array
        //if (pose_ind != -1) {
        drone_pose_gt = utils::extract_pose_from_pose_array_msg(gt_msg, pose_ind);
        // } else {
        //     // Otherwise use it directly
        //     drone_pose_gt = gt_msg->pose;
        // }

        // Update state with position and orientation
        state_obj_gt.setPos(Eigen::Vector3d(drone_pose_gt.position.x, drone_pose_gt.position.y, drone_pose_gt.position.z));
        Eigen::Quaterniond att_q(drone_pose_gt.orientation.w, drone_pose_gt.orientation.x, drone_pose_gt.orientation.y, drone_pose_gt.orientation.z);
        state_obj_gt.setAtt(utils::convert_quaternion_eigen_to_tf(att_q));

        // Publish the transform (broadcast TF)
        utils::broadcast_tf(time, "ground_truth", name_frame_child + "_gt", state_obj_gt.getPos(), att_q, tf_broadcaster); //utils::convert_quaternion_tf_to_eigen(state_obj_gt.getAtt())

        // If pixhawk_pose is provided, set the GPS home flag
        // if (pixhawk_pose != nullptr) {
        //     pixhawk_pose->set_flag_gps_home();
        // }

        return state_obj_gt;
    }

    // MATH
    // Helper function to calculate the trace of a 3x3 matrix
    float getTrace(const tf2::Matrix3x3& matrix) {
        return matrix[0][0] + matrix[1][1] + matrix[2][2];
    }

    // STRING AND FILE HANDLING
    int extract_id_from_name(const std::string& input) {
        size_t underscorePos = input.rfind('_');

        if (underscorePos != std::string::npos) {
            std::string lastNumber = input.substr(underscorePos + 1);
            return std::stoi(lastNumber);
        } else {
            return -1;
        }
    }

    // Helper function to split a string by a delimiter and convert to float
    std::vector<float> splitAndConvert(const std::string& s, char delimiter) {
        std::vector<float> result;
        std::istringstream ss(s);
        std::string token;

        while (std::getline(ss, token, delimiter)) {
            result.push_back(std::stof(token));
        }

        return result;
    }
    

    // OTHER HANDLING
    geometry_msgs::msg::Pose extract_pose_from_pose_array_msg(const geometry_msgs::msg::PoseArray &pose_array, size_t index) {
        // Check if the index is valid
        if (index >= pose_array.poses.size()) {
            throw std::out_of_range("Index is out of bounds for the PoseArray.");
        }
        
        // Return the pose at the given index
        return pose_array.poses[index];
    }

    // CONVERSIONS
    Eigen::Vector3d convert_vec_floats_to_eigen(const std::vector<float>& float_vector) {
        Eigen::Vector3d eigen_vector;

        // Check if the vector has exactly 3 elements
        if (float_vector.size() != 3) {
            std::cerr << "Error: Vector does not contain exactly 3 elements." << std::endl;
            return eigen_vector;
        }

        // Convert std::vector<float> to Eigen::Vector3d
        eigen_vector << static_cast<double>(float_vector[0]),
                        static_cast<double>(float_vector[1]),
                        static_cast<double>(float_vector[2]);

        return eigen_vector;                
    }


    geometry_msgs::msg::Pose convert_state_to_pose_msg(const droneState::State& state) {
        geometry_msgs::msg::Pose pose_msg;

        // Set position
        pose_msg.position.x = state.getPos().x();
        pose_msg.position.y = state.getPos().y();
        pose_msg.position.z = state.getPos().z();

        // Set orientation
        pose_msg.orientation.x = state.getAtt().x();
        pose_msg.orientation.y = state.getAtt().y();
        pose_msg.orientation.z = state.getAtt().z();
        pose_msg.orientation.w = state.getAtt().w();

        return pose_msg;
    }

    droneState::State convert_tf_stamped_msg_to_state(const geometry_msgs::msg::TransformStamped& pose_msg, 
                                                std::string frame, 
                                                droneState::CS_type cs_type, 
                                                Eigen::Vector3d vel){
        // Extract position from pose_msg
        Eigen::Vector3d pos = Eigen::Vector3d(pose_msg.transform.translation.x, 
                                              pose_msg.transform.translation.y, 
                                              pose_msg.transform.translation.z);
        
        // Extract orientation from pose_msg
        tf2::Quaternion att(pose_msg.transform.rotation.x, 
                            pose_msg.transform.rotation.y, 
                            pose_msg.transform.rotation.z, 
                            pose_msg.transform.rotation.w);

        // Set position and orientation in state
        droneState::State state = droneState::State(frame, cs_type, pos, att, vel);

        // state.setPos(position);
        // state.setAtt(att);

        return state;
    }

    // tf2::Quaternion convert_rvec_to_quaternion(const cv::Vec3d& rvec) {
    //     // Convert rotation vector to rotation matrix
    //     cv::Mat rotMat;
    //     cv::Rodrigues(rvec, rotMat);

    //     // Convert OpenCV matrix to tf2 matrix
    //     tf2::Matrix3x3 tf2_rotMat(
    //         rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
    //         rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
    //         rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2)
    //     );

    //     // Create a quaternion from the rotation matrix
    //     tf2::Quaternion tf2_quaternion;
    //     tf2_rotMat.getRotation(tf2_quaternion);

    //     return tf2_quaternion;
    // }

    // Convert a rotation vector to a rotation matrix in Eigen::Vector3d format
    Eigen::Matrix3d convert_rvec_to_rotmat(const Eigen::Vector3d& rvec) {
        // Convert Eigen vector to a tf2 quaternion
        tf2::Quaternion quat;
        quat.setRPY(rvec[0], rvec[1], rvec[2]);
        
        // Convert tf2 quaternion to an eigen rotation matrix
        tf2::Matrix3x3 tf2_rotMat(quat);
        Eigen::Matrix3d eigen_rotMat;

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                eigen_rotMat(row, col) = tf2_rotMat[row][col];
            }
        }
        
        return eigen_rotMat;
    }

    tf2::Quaternion convert_quaternion_eigen_to_tf(const Eigen::Quaterniond& q){
        // Create a tf2::Quaternion and set its components based on Eigen::Quaterniond
        tf2::Quaternion q_tf2;
        q_tf2.setValue(q.x(), q.y(), q.z(), q.w());  // tf2 uses (x, y, z, w) format

        return q_tf2;
    }

    Eigen::Quaterniond convert_quaternion_tf_to_eigen(const tf2::Quaternion& q) {
        // Create an Eigen::Quaterniond and set its components based on tf2::Quaternion
        Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());  // Eigen uses (w, x, y, z) format

        return q_eigen;
    }

    // PX4
    int extract_instance_from_connection(const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& pub_vehicle_command) {
        // Get the topic name from the publisher
        std::string topic_name = pub_vehicle_command->get_topic_name();

        // Split the topic name based on '/' and extract the namespace (second part)
        size_t first_slash = topic_name.find('/');
        size_t second_slash = topic_name.find('/', first_slash + 1);
        std::string namespace_str = topic_name.substr(first_slash + 1, second_slash - first_slash - 1);

        // Split the namespace by '_' and extract the instance number (second part)
        size_t underscore_pos = namespace_str.find('_');
        std::string instance_str = namespace_str.substr(underscore_pos + 1);

        // Convert to integer and return
        return std::stoi(instance_str);
    }

    // Send a command to Pixhawk
    void publish_vehicle_command(uint32_t command, const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& pub_vehicle_command,
                                 const rclcpp::Time& timestamp, double param1, double param2, double param3, double param4, double param5, double param6, double param7) {
        
        // Extract the instance number
        uint8_t instance_num = utils::extract_instance_from_connection(pub_vehicle_command) + 1;

        // Create the VehicleCommand message
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;

        // Set the target system to instance_num (as per PX4 documentation)
        msg.target_system = instance_num;
        msg.target_component = 1;  // Typically 1 for vehicle command
        msg.source_system = 1;     // Typically 1 for source system
        msg.source_component = 1;  // Typically 1 for source component
        msg.from_external = true;  // Mark as external
        msg.timestamp = static_cast<unsigned long>(timestamp.nanoseconds() / 1000);

        // Publish the message
        pub_vehicle_command->publish(msg);
    }

    // Set GPS origin
    void set_origin(const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr& pub_vehicle_command, const rclcpp::Time& timestamp, double lat, double lon, double alt) {
        utils::publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
                                pub_vehicle_command, timestamp, 0.0, 0.0, 0.0, 0.0, lat, lon, alt);
    }

    // Send offboard heartbeat signal
    void publish_offboard_control_heartbeat_signal(
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode, 
        const std::string& what_control, 
        uint64_t timestamp)
    {
        // Create the message
        px4_msgs::msg::OffboardControlMode msg;

        // Set all control flags to false initially
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;

        // Set the control flag based on what_control
        if (what_control == "pos") {
            msg.position = true;
        } else if (what_control == "vel") {
            msg.velocity = true;
        } else if (what_control == "accel") {
            msg.acceleration = true;
        }

        // Set the timestamp
        msg.timestamp = timestamp;

        // Publish the message
        pub_offboard_mode->publish(msg);
    }


    


} // namespace utils