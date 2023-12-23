#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <utility>
#include <rclcpp/rclcpp.hpp>
//#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>

//#include <nav2_msgs/action/follow_path.hpp>

#define MAX_SPEED 0.3
#define ZERO 0.0
#define MAX_ALLOWED_ERROR 0.1

using namespace std::chrono_literals;
using  namespace std::placeholders;

typedef struct {
    double x;
    double y;
    double theta; // radians or degrees
} pose;

typedef std::vector<pose> waypoints;

namespace smore {

class FeedbackController : public rclcpp::Node {

public:
    FeedbackController(waypoints path) : rclcpp::Node("smore_Feedback_Controller"), 
    current_pose_({0.0f, 0.0f, 0.0f}), path_(path) {
        rclcpp::SubscriptionOptions options_odom;
        rclcpp::SubscriptionOptions options_pose;

        cb_group_odom_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_pose_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        options_odom.callback_group = cb_group_odom_;
        options_pose.callback_group = cb_group_pose_;

        sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("/smore/goal_position", 10, 
            std::bind(&FeedbackController::TargetPoseCallback, this, _1), options_pose);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/smore/odometry", 10, 
            std::bind(&FeedbackController::OdometryCallback, this, _1), options_odom);
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/smore/cmd_vel", 10);
        
        //FollowPath();
    }

private:

    void FollowPath() {
        for(auto point : path_){
            MoveTo(point.x, point.y, point.theta);
        }
    }

    void TargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        double x_goal = msg->position.x;
        double y_goal = msg->position.y;
        double theta_goal = msg->orientation.z;
        MoveTo(x_goal, y_goal, theta_goal);
    }
    

    /**
     * @brief Normalizes angle to 
     * 
     * @param angle 
     * @return double radians value between (-pi, pi)
     */
    double Normalize(double angle) {
        return tf2Atan2(tf2Sin(angle), tf2Cos(angle));
    }

    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        current_pose_.theta = yaw;
        RCLCPP_INFO(this->get_logger(), "Odometry:\n position:\n \tx: %4f\n\ty: %4f\norientation:\n \tz: %4f", 
        msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        rclcpp::sleep_for(100ms);
    }

    void MoveTo(double x_goal, double y_goal, double theta_goal_degrees) {
        double rho = INFINITY;
        
        double theta_goal_radians = tf2Radians(theta_goal_degrees);

        while(rho > MAX_ALLOWED_ERROR) {
            
            double dx = x_goal - current_pose_.x;
            double dy = y_goal - current_pose_.y;
            double theta = current_pose_.theta ;
            
            // RCLCPP_INFO(this->get_logger(), "dx: %f, dy: %f", dx, dy);
            // Get polar coordinates
            rho = tf2Sqrt(tf2Pow(dx, 2) + tf2Pow(dy, 2));
            if(rho <= MAX_ALLOWED_ERROR) {
                break;
            }
            double alpha = Normalize(tf2Atan2(dy,dx) - theta);
            double beta = Normalize(-theta_goal_radians - alpha);
            
            RCLCPP_INFO(this->get_logger(), "segment: %f", rho);
            RCLCPP_INFO(this->get_logger(), "target:\nposiotion\nx: %f\n, y: %f\norientation: %f", x_goal, y_goal,theta_goal_radians);
            //RCLCPP_INFO(this->get_logger(), "c_x: %f, c_y: %f, c_theta: %f", current_pose_.x, current_pose_.y, current_pose_.theta);
            RCLCPP_INFO(this->get_logger(), "alpha: %f, beta: %f", alpha, beta);
            double linear_v = k_rho * rho;
            double angular_w = k_alpha * alpha + k_beta * beta;
            double abs_v = std::abs(linear_v);
            linear_v = (linear_v / abs_v) * MAX_SPEED;
           
            angular_w = (angular_w / abs_v ) * MAX_SPEED;
            RCLCPP_INFO(this->get_logger(), "linear: %f, angular: %f", linear_v, angular_w);
            PublishVelocities(linear_v, angular_w);
            rclcpp::sleep_for(10ms);
            //break;
        }
        PublishVelocities(0.0, 0.0);
    }

    void PublishVelocities(double linear_x, double angular_z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        publisher_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::CallbackGroup::SharedPtr cb_group_odom_;
    rclcpp::CallbackGroup::SharedPtr cb_group_pose_;
    pose current_pose_;
    const double k_rho{0.3};
    const double k_alpha{0.8};
    const double k_beta{-0.15};
    waypoints path_;

};

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    waypoints path {{1,-1,0}};
    std::shared_ptr<smore::FeedbackController> controller_node = 
        std::make_shared<smore::FeedbackController>(path);

    rclcpp::executors::MultiThreadedExecutor  executor;
    executor.add_node(controller_node);
    
    executor.spin();
    // rclcpp::spin(std::make_shared<smore::FeedbackController>(path));
    rclcpp::shutdown();
    return 0;
}
