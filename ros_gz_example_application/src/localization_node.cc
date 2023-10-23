#include "localization_node.hpp"

using  namespace std::placeholders;
namespace smore {

Localization::Localization(): rclcpp::Node("smore_Localization") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/smore/odometry", 10, 
        std::bind(&Localization::OdometryCallback, this, _1));
}

void Localization::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odometry:\n position:\n \tx: %4f\n\ty: %4f\norientation:\n \tz: %4f", 
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
}
    
} // namespace smore


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smore::Localization>());
  rclcpp::shutdown();
  return 0;
}