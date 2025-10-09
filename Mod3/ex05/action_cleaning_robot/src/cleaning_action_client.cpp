#include <rclcpp/rclcpp.hpp>
#include <cleaning_action/action/cleaning_task.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using CleaningTask = cleaning_action::action::CleaningTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<CleaningTask>;

class CleaningActionClient : public rclcpp::Node
{
public:
    CleaningActionClient() : Node("cleaning_action_client")
    {
        client_ = rclcpp_action::create_client<CleaningTask>(this, "cleaning_task");
        RCLCPP_INFO(this->get_logger(), "Cleaning Action Client started");
    }

    void send_home(double x, double y)
    {
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal = CleaningTask::Goal();
        goal.task_type = "return_home";
        goal.target_x = x;
        goal.target_y = y;
        send_goal(goal, "Return home");
    }

    void send_clean(double x, double y, double size)
    {
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal = CleaningTask::Goal();
        goal.task_type = "clean_square";
        goal.area_size = size;
        goal.target_x = x;
        goal.target_y = y;
        send_goal(goal, "Square cleaning");
    }

private:
    rclcpp_action::Client<CleaningTask>::SharedPtr client_;

    void send_goal(const CleaningTask::Goal &goal, const std::string &task_name)
    {
        auto send_goal_options = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr & goal_handle)  {
                if (!goal_handle) {
                    RCLCPP_ERROR(rclcpp::get_logger("client"), "Task was rejected");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("client"), "Task accepted");
                }
            };
        
        send_goal_options.feedback_callback =
            [task_name](GoalHandle::SharedPtr, const std::shared_ptr<const CleaningTask::Feedback> feedback) {
                RCLCPP_INFO(rclcpp::get_logger("client"), 
                           "%s: %d%% complete, cleaned %d points, at (%.2f, %.2f)",
                           task_name.c_str(), feedback->progress_percent, 
                           feedback->current_cleaned_points, feedback->current_x, feedback->current_y);
            };
        
        send_goal_options.result_callback =
            [task_name](const GoalHandle::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(rclcpp::get_logger("client"), 
                                   "%s succeeded! Cleaned %d points, distance: %.2f",
                                   task_name.c_str(), result.result->cleaned_points, result.result->total_distance);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(rclcpp::get_logger("client"), "%s was aborted", task_name.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(rclcpp::get_logger("client"), "%s was canceled", task_name.c_str());
                        break;
                    default:
                        RCLCPP_ERROR(rclcpp::get_logger("client"), "%s unknown result", task_name.c_str());
                        break;
                }
            };

        RCLCPP_INFO(this->get_logger(), "Sending goal: %s", task_name.c_str());
        client_->async_send_goal(goal, send_goal_options);
        
        // Wait a bit between goals
        std::this_thread::sleep_for(5s);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<CleaningActionClient>();
    if (argc < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
        "Usage: ros2 run action_cleaning_robot cleaning_action_client <command> <x> <y> <size>\n(Ignore <size> for return_home command)");
        return 1;
    }
    std::string command = std::string(argv[1]);
    if (command == std::string("clean_square"))
    {
        client_node->send_clean(
                   std::stod(argv[2]),
                   std::stod(argv[3]),
                   std::stod(argv[4])
                  );
    }
    else if (command == std::string("return_home"))
    {
        client_node->send_home(
                   std::stod(argv[2]),
                   std::stod(argv[3])
                  );
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
        "Unknown command: %s (Available: return_home, clean_square)", command.c_str());
        return 1;
    }
    
    // Keep running to receive results
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}
