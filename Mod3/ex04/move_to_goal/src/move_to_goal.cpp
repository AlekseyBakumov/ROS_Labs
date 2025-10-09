#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

#define DISTANCE_TOLERANCE 0.1
#define ANGLE_TOLERANCE 0.05
#define LINEAR_P_GAIN 1.5
#define ANGULAR_P_GAIN 6.0
#define MAX_LINEAR_VELOCITY 2.0
#define MAX_ANGULAR_VELOCITY 2.0
#define CONTROL_LOOP_FREQUENCY 10
#define TURTLESIM_MIN_X 0.0
#define TURTLESIM_MAX_X 11.0
#define TURTLESIM_MIN_Y 0.0
#define TURTLESIM_MAX_Y 11.0

class MoveToGoal : public rclcpp::Node
{
public:
    MoveToGoal(double goal_x, double goal_y, double goal_theta)
    : Node("move_to_goal")
    {
        goal_x_ = goal_x;
        goal_y_ = goal_y;
        goal_theta_ = goal_theta;
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&MoveToGoal::pose_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROL_LOOP_FREQUENCY),
            std::bind(&MoveToGoal::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Moving turtle to goal: (%.2f, %.2f, %.2f)", 
                   goal_x_, goal_y_, goal_theta_);
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void control_loop()
    {
        if (!pose_received_) return;

        auto msg = geometry_msgs::msg::Twist();
        
        // Calculate distance
        double dx = goal_x_ - current_pose_.x;
        double dy = goal_y_ - current_pose_.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle
        double target_angle = std::atan2(dy, dx);
        double angle_error = target_angle - current_pose_.theta;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        
        // Final orientation
        double final_angle_error = goal_theta_ - current_pose_.theta;
        while (final_angle_error > M_PI) final_angle_error -= 2 * M_PI;
        while (final_angle_error < -M_PI) final_angle_error += 2 * M_PI;

        // Control logic
        if (distance > DISTANCE_TOLERANCE) {
            // Rotate
            if (std::abs(angle_error) > ANGLE_TOLERANCE) {
                msg.angular.z = ANGULAR_P_GAIN * angle_error;
                msg.linear.x = 0.0;
            } else {
                // Move
                msg.linear.x = LINEAR_P_GAIN * distance;
                msg.angular.z = ANGULAR_P_GAIN * angle_error;
            }
        } else {
            // Rotate
            if (std::abs(final_angle_error) > ANGLE_TOLERANCE) {
                msg.angular.z = ANGULAR_P_GAIN * final_angle_error;
                msg.linear.x = 0.0;
            } else {
                // Finish
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                rclcpp::shutdown();
                return;
            }
        }
        
        msg.linear.x = std::clamp(msg.linear.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
        msg.angular.z = std::clamp(msg.angular.z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
        
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;
    
    double goal_x_, goal_y_, goal_theta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                    "Usage: ros2 run move_to_goal move_to_goal <x> <y> <theta>");
        return 1;
    }
    
    try {
        double goal_x = std::stod(argv[1]);
        double goal_y = std::stod(argv[2]);
        double goal_theta = std::stod(argv[3]);
        
        if (goal_x < TURTLESIM_MIN_X || goal_x > TURTLESIM_MAX_X || 
            goal_y < TURTLESIM_MIN_Y || goal_y > TURTLESIM_MAX_Y) {
            RCLCPP_WARN(rclcpp::get_logger("main"), 
                       "Coordinates outside turtlesim range (%.1f-%.1f, %.1f-%.1f)",
                       TURTLESIM_MIN_X, TURTLESIM_MAX_X, TURTLESIM_MIN_Y, TURTLESIM_MAX_Y);
        }
        
        auto node = std::make_shared<MoveToGoal>(goal_x, goal_y, goal_theta);
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                    "Error parsing arguments: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}