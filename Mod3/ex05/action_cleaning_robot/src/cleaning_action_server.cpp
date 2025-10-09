#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cleaning_action/action/cleaning_task.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using CleaningTask = cleaning_action::action::CleaningTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<CleaningTask>;

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

class CleaningActionServer : public rclcpp::Node
{
public:
    CleaningActionServer() : Node("cleaning_action_server")
    {
        // Action server
        action_server_ = rclcpp_action::create_server<CleaningTask>(
            this, "cleaning_task",
            std::bind(&CleaningActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CleaningActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CleaningActionServer::handle_accepted, this, std::placeholders::_1));
        
        // Publisher and subscriber
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&CleaningActionServer::pose_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Cleaning Action Server started");
    }

private:
    rclcpp_action::Server<CleaningTask>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CleaningTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal: %s", goal->task_type.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&CleaningActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CleaningTask::Result>();
        auto feedback = std::make_shared<CleaningTask::Feedback>();
        
        // Wait for pose data
        while (!pose_received_) {
            std::this_thread::sleep_for(100ms);
        }
        
        double start_x = current_pose_.x;
        double start_y = current_pose_.y;
        
        if (goal->task_type == "clean_square") {
            clean_square(goal_handle, feedback, result, goal->target_x, goal->target_y, goal->area_size);
        }
        else if (goal->task_type == "return_home") {
            return_home(goal_handle, feedback, result, goal->target_x, goal->target_y);
        }
        
        goal_handle->succeed(result);
    }

    void clean_square(const std::shared_ptr<GoalHandle> goal_handle,
                     std::shared_ptr<CleaningTask::Feedback> feedback,
                     std::shared_ptr<CleaningTask::Result> result,
                     double start_x, double start_y, double size)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to target: %.2fx%.2f", start_x, start_y);

        move_to_point(start_x, start_y);

        RCLCPP_INFO(this->get_logger(), "Cleaning square: %.2fx%.2f", size, size);

        int total_points = 8;
        int points_cleaned = 0;
        
        float current_x = start_x;
        float current_y = start_y;
        bool up = false;

        float move_x = size;

        // Simple square cleaning pattern
        for (int i = 0; i < total_points && rclcpp::ok(); i++) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                return;
            }
            
            // Move in a simple pattern
            move_to_point(current_x, current_y);
            
            
            if (up) current_y += size / 4.0f;
            else
            { 
                current_x += move_x;
                move_x *= -1;
            }
            up = !up;

            points_cleaned++;
            
            // Update feedback
            feedback->progress_percent = (i + 1) * 100 / total_points;
            feedback->current_cleaned_points = points_cleaned;
            feedback->current_x = current_pose_.x;
            feedback->current_y = current_pose_.y;
            goal_handle->publish_feedback(feedback);
            
            std::this_thread::sleep_for(500ms);
        }
        
        result->success = true;
        result->cleaned_points = points_cleaned;
        result->total_distance = size * 4; // Approximate
    }

    void return_home(const std::shared_ptr<GoalHandle> goal_handle,
                    std::shared_ptr<CleaningTask::Feedback> feedback,
                    std::shared_ptr<CleaningTask::Result> result,
                    double home_x, double home_y)
    {
        RCLCPP_INFO(this->get_logger(), "Returning home: (%.2f, %.2f)", home_x, home_y);
        
        move_to_point(home_x, home_y);
        
        feedback->progress_percent = 100;
        feedback->current_cleaned_points = 0;
        feedback->current_x = current_pose_.x;
        feedback->current_y = current_pose_.y;
        goal_handle->publish_feedback(feedback);
        
        result->success = true;
        result->cleaned_points = 0;
        result->total_distance = calculate_distance(current_pose_.x, current_pose_.y, home_x, home_y);
    }

    void move_to_point(double target_x, double target_y)
    {
        auto msg = geometry_msgs::msg::Twist();
        double distance = 1;

        while (distance > 0.1)        
        {
            // Calculate distance
            double dx = target_x - current_pose_.x;
            double dy = target_y - current_pose_.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            // Calculate angle
            double target_angle = std::atan2(dy, dx);
            double angle_error = target_angle - current_pose_.theta;
            
            // Normalize angle to [-pi, pi]
            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;

            // Control logic
            if (distance > DISTANCE_TOLERANCE) 
                {
                // Rotate
                if (std::abs(angle_error) > ANGLE_TOLERANCE) 
                    {
                        msg.angular.z = ANGULAR_P_GAIN * angle_error;
                        msg.linear.x = 0.0;
                    } 
                else 
                    {
                        // Move
                        msg.linear.x = LINEAR_P_GAIN * distance;
                        msg.angular.z = ANGULAR_P_GAIN * angle_error;
                    }
                }
            else 
                {
                    // Finish
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    publisher_->publish(msg);
                    break;
                }
            
            msg.linear.x = std::clamp(msg.linear.x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
            msg.angular.z = std::clamp(msg.angular.z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
            
            publisher_->publish(msg);
            std::this_thread::sleep_for(100ms);
        }
        
        
        // Stop
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CleaningActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
