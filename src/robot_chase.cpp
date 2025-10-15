#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

// Ref:
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase_node") {
    RCLCPP_INFO(get_logger(), "Robot Chase Node is Ready");
    target_frame_ =
        declare_parameter<std::string>("target_frame", "morty/base_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               std::bind(&RobotChase::on_timer, this));
  }

private:
  void on_timer() {
    std::string fromFrame = target_frame_.c_str();
    std::string toFrame = "rick/base_link";

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "Cound not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), e.what());
      return;
    }
    double t_x = t.transform.translation.x;
    double t_y = t.transform.translation.y;
    double error_distance = std::hypot(t_x, t_y);
    double error_yaw = std::atan2(t_y, t_x);

    const double kp_distance = 0.5;
    const double kp_yaw = 0.8;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = kp_distance * error_distance;
    cmd.angular.z = kp_yaw * error_yaw;
    pub_->publish(cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listen_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}