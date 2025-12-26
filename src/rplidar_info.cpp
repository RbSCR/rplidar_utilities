
#include "rclcpp/rclcpp.hpp"

class RplidarInfo : public rclcpp::Node
{
public:
    RplidarInfo() : Node("rplidar_info")
    {
      RCLCPP_INFO(this->get_logger(), "To be implemented");
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RplidarInfo>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
