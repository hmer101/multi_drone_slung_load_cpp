#ifndef PIXHAWK_H
#define PIXHAWK_H

#include <string>

#include <rclcpp/rclcpp.hpp>
//#include <tf2_ros/transform_listener.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include "std_msgs/msg/string.hpp"

//#include "multi_drone_slung_load_interfaces/msg/phase.hpp"
// #include "multi_drone_slung_load_cpp/State.h"
// #include "multi_drone_slung_load_cpp/utils.h"


class Pixhawk : public rclcpp::Node {
public:
    Pixhawk();
    //~Pixhawk();

private:
    // PARAMETERS
    std::string ns_; // Namespace of the node
    int id_; // ID of the pixhawk this node is connected to

    // VARIABLES
  
};

#endif // PIXHAWK_H