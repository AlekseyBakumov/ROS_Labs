#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TextToCmdVel : public rclcpp::Node
{
  public:
    TextToCmdVel()
    : Node("text_to_cmd_vel")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cmd_text", 10, std::bind(&TextToCmdVel::twist_callback, this, _1));
    }

  private:
    void twist_callback(const std_msgs::msg::String & msg) const
    {
      auto message = geometry_msgs::msg::Twist();
      int command_ = -1;
      if (strcmp(msg.data.c_str(), "turn_left")     == 0) command_ = 1;
      if (strcmp(msg.data.c_str(), "turn_right")    == 0) command_ = 2;
      if (strcmp(msg.data.c_str(), "move_forward")  == 0) command_ = 3;
      if (strcmp(msg.data.c_str(), "move_backward") == 0) command_ = 4;
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      switch (command_)
      {
      case 1:
        message.angular.z = -0.2;
        break;
        
      case 2:
        message.angular.z = -0.2;
        break;
        
      case 3:
        message.linear.x = 1.0;
        break;
        
      case 4:
        message.linear.x = -1.0;
        break;
      
      default:
        RCLCPP_INFO(this->get_logger(), "I heard unknown message: '%s'", msg.data.c_str());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "I heard message: '%s'", msg.data.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing Twist: linear.x = '%.2f', angular.z = '%.2f'",
                                                         message.linear.x,  message.angular.z);
      publisher_->publish(message);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
