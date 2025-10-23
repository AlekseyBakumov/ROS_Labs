#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
  DynamicFrameBroadcaster()
  : Node("dynamic_frame_tf2_broadcaster")
  {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this));
      this->declare_parameter("radius", 4.0f);
      this->declare_parameter("direction", 1);
      base_target_x_ = this->declare_parameter("base_x", 1.0);
      base_target_y_ = this->declare_parameter("base_y", 8.0);
      switch_threshold_ = this->declare_parameter("switch_threshold", 1.0f);
      current_target_index_ = 0;
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      publisher_ = this->create_publisher<std_msgs::msg::String>("/current_target", 1);
  }

private:
  void broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    float radius = 1.0f;
    int direction = 1;

    std_msgs::msg::String msg;

    this->get_parameter("radius", radius);
    this->get_parameter("direction", direction);

    this->get_parameter("base_x", base_target_x_);
    this->get_parameter("base_y", base_target_y_);

    this->get_parameter("switch_threshold", switch_threshold_);

    if (radius <= 0.0f)
    {
      RCLCPP_WARN(this->get_logger(), "Incorrect radius (%f). Radius value set to 1", radius);
      radius = 1.0f;
      this->set_parameter(rclcpp::Parameter("radius", 1.0f));
    }
    if (direction != 1 && direction != -1)
    {
      RCLCPP_WARN(this->get_logger(), "Incorrect direction (%d). Direction value set to 1", direction);
      direction = 1;
      this->set_parameter(rclcpp::Parameter("direction", 1));
    }

    geometry_msgs::msg::TransformStamped target_t_;
    float distance_to_target_ = 99999.9f;

    try {
      target_t_ = tf_buffer_->lookupTransform(
        targets[current_target_index_].c_str(), "turtle2",
        tf2::TimePointZero);
        distance_to_target_ = std::sqrt(
          std::pow(target_t_.transform.translation.x,2) +
          std::pow(target_t_.transform.translation.y,2)
    );
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          targets[current_target_index_].c_str(), "turtle2", ex.what());
        }

    if (distance_to_target_ < switch_threshold_)
    {
      if (++current_target_index_ >= (int)(targets.size())) current_target_index_ = 0;
      msg.data = targets[current_target_index_];
      publisher_->publish(msg);
    }

    double x = now.seconds() * PI / 10 * direction;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    t.transform.translation.x = radius * sin(x);
    t.transform.translation.y = radius * cos(x);
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  
    t.header.frame_id = "turtle3";
    t.child_frame_id = "carrot2";
    tf_broadcaster_->sendTransform(t);
    
    t.transform.translation.x = this->base_target_x_;
    t.transform.translation.y = this->base_target_y_;
    t.header.frame_id = "world";
    t.child_frame_id = "static_target";
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  float base_target_x_;
  float base_target_y_;
  float switch_threshold_;

  const std::array<std::string, 3> targets = {
    "carrot1",
    "carrot2",
    "static_target"
  };
  int current_target_index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
