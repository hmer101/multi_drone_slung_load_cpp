#include <rclcpp/rclcpp.hpp>

#include "multi_drone_slung_load_cpp/pixhawk.h"
//#include "multi_drone_slung_load_cpp/State.h"
#include "multi_drone_slung_load_cpp/utils.h"


Pixhawk::Pixhawk() : Node("pixhawk", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    this->ns_ = this->get_namespace();
    this->id_ = utils::extract_id_from_name(this->ns_);

    // Print info
    RCLCPP_INFO(this->get_logger(), "PIXHAWK NODE %d", this->id_);

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pixhawk>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}