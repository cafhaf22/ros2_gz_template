#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define MAX_SPEED 0.3
#define ZERO 0.0

//                angle, radius
// if angle = 0   angle, segment              
typedef std::pair<int, int> waypoint;

namespace utils
{
    float DegreeToRadians (float degrees) {
        return (degrees * (M_PI / 180.0));
    }
} // namespace utils


namespace smore
{
class VelocityController : public rclcpp::Node {

public:
    VelocityController(std::vector<waypoint>path) : rclcpp::Node("smore_Openloop_control"), path_(path) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/smore/cmd_vel", 10);
        FollowPath();
        
    }
    void FollowPath() {
        for(auto& movement : path_) {
            if(movement.first == 0) {
                // if angle == 0 then is a straigth line
                foward(movement.second);
            } else {
                // it is a turn
                if(movement.second > 0) {
                    // if radius is positive then turn left
                    left(movement.first, movement.second);
                } else {
                    // turn right
                    right(movement.first, movement.second);
                }
            }
        }

    }


private:
    void foward(int segment) {
        std::chrono::seconds movement_time = CalculateTime(segment);
        move(MAX_SPEED, ZERO);
        rclcpp::sleep_for(movement_time);
    }

    void left(float angle = 90.0, float radius = 1.0) {

        float rotational_speed = CalculateRotationalSpeed(radius);
        // calculate the segment that the robot will be moving
        // segment = radius[radians] * angle[radians]
        int segment = std::abs(radius) * std::abs(utils::DegreeToRadians(angle));
        std::chrono::seconds movement_time = CalculateTime(segment);
        move(MAX_SPEED, rotational_speed);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(movement_time));    
    }

    void right(float angle = 90.0, float radius = 1.0) {
        float rotational_speed = CalculateRotationalSpeed(radius);
        int segment = std::abs(radius) * std::abs(utils::DegreeToRadians(angle));
        std::chrono::seconds movement_time = CalculateTime(segment);
        move(MAX_SPEED, rotational_speed);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(movement_time));

    }
    
    void move(float linear_x=0.0, float angular_z = 0.0) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        publisher_->publish(msg);
    }

    std::chrono::seconds CalculateTime(int segment) {
        int dt = int(segment / MAX_SPEED);
        std::chrono::seconds sec = std::chrono::duration<long int>(dt);
        return sec;
    }

    float CalculateRotationalSpeed(float radius) {
        return (MAX_SPEED / radius);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::vector<waypoint>path_;


};

} // namespace smore

int main(int argc, char * argv[])
{
    /*right(), left(), straight(), left(), left(radius=0.5), right(radius=0.5),
        straight(), left(angle=180,radius=0.5), right(), straight()]*/
    std::vector<waypoint> path {{90, -1}, {0, 1}, {90, 1}};
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smore::VelocityController>(path));
    rclcpp::shutdown();
    return 0;
}

