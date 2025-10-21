#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

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
  }

private:
  void broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    float radius = 1.0f;
    int direction = 1;

    this->get_parameter("radius", radius);
    this->get_parameter("direction", direction);

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
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
