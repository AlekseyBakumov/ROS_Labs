#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/full_name_sum_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_name_strings_client FName Name LName");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_name_strings_client");
  rclcpp::Client<tutorial_interfaces::srv::FullNameSumService>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::FullNameSumService>("add_name_strings");

  auto request = std::make_shared<tutorial_interfaces::srv::FullNameSumService::Request>();
  request->first_name = std::string(argv[1]);
  request->name       = std::string(argv[2]);
  request->last_name  = std::string(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", result.get()->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_name_strings");
  }

  rclcpp::shutdown();
  return 0;
}
