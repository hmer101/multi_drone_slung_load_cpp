#include "rclcpp/rclcpp.hpp"

class SimTimeNode : public rclcpp::Node
{
public:
    SimTimeNode()
    : Node("sim_time_node", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // Ensure the node uses simulation time
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Current time: %.2f", this->now().seconds());
            }
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimTimeNode>());
    rclcpp::shutdown();
    return 0;
}