#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

#define STOP 0.0

using namespace std::chrono_literals;



namespace smore
{

class ConvertVelocities: public rclcpp::Node {

public:
    ConvertVelocities(): rclcpp::Node("smore_ConvertVelocities"), 
    linear_(0.0), angular_(0.0) {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/smore/cmd_vel", 10, std::bind(
            &ConvertVelocities::TwistCallback, this, std::placeholders::_1
        ));

        timer_ = this->create_wall_timer(2s, std::bind(
            &ConvertVelocities::TimerCallback, this));
    }


private:

    void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_ = msg->linear.x;
        angular_ = msg->angular.z;
    }

    void TimerCallback(){
        right_wheel_speed_ = ((2 * linear_) + (angular_ * wheel_distance_)) / (2 * wheel_radious_);
        left_wheel_speed_ = ((2 * linear_) - (angular_ * wheel_distance_)) / (2 * wheel_radious_);
        RCLCPP_INFO(this->get_logger(), "Vr = %f\nVl = %f", right_wheel_speed_, left_wheel_speed_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    const float wheel_distance_ = 0.224;
    const float wheel_radious_ = 0.12;
    float linear_;
    float angular_;
    float right_wheel_speed_;
    float left_wheel_speed_;

};

} // namespace smore


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smore::ConvertVelocities>());
  rclcpp::shutdown();
  return 0;
}
