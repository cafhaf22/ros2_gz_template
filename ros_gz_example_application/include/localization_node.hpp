#ifndef INCLUDE_LOCALIZATION_NODE_HPP_
#define INCLUDE_LOCALIZATION_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace smore {

class Localization : public rclcpp::Node {

public:
    explicit Localization();


private:
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

};


} // namespace smore

#endif // INCLUDE_LOCALIZATION_NODE_HPP_

