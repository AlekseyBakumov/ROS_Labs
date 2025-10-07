#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/full_name_sum_service.hpp"

#include <memory>
#include <string>

void add(const std::shared_ptr<tutorial_interfaces::srv::FullNameSumService::Request> request,
          std::shared_ptr<tutorial_interfaces::srv::FullNameSumService::Response>      response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nFirst name: %s\nName: %s\nLast name: %s\n",
                request->first_name.c_str(),
                request->name.c_str(),
                request->last_name.c_str());
  response->full_name = request->first_name +
                        std::string(" ") + request->name +
                        std::string(" ") + request->last_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_name_strings_server");

  rclcpp::Service<tutorial_interfaces::srv::FullNameSumService>::SharedPtr service =
    node->create_service<tutorial_interfaces::srv::FullNameSumService>("add_name_strings", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add name strings.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
