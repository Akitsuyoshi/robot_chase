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
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&RobotChase::on_timer, this));
  }

private:
  void on_timer() {
    std::string fromFrame = target_frame_;
    std::string toFrame = "rick/base_link";

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), e.what());
      return;
    }
    double t_x = t.transform.translation.x;
    double t_y = t.transform.translation.y;
    double error_distance = std::hypot(t_x, t_y);
    double error_yaw = std::atan2(t_y, t_x);

    // Linear vars
    const double kp_distance = 0.7;
    const double max_linear = 1.0;

    // Angular vars
    const double kp_yaw = 1.0;
    const double max_angular = M_PI / 3;

    // Stop and bump vars
    const double bump_distance = 0.36;

    geometry_msgs::msg::Twist cmd;
    if (error_distance < bump_distance) {
      cmd.linear.x = 0.0;
    } else if (std::abs(error_yaw) > M_PI / 2) {
      cmd.linear.x = 0.0;
    } else {
      double alignment_fact = std::cos(error_yaw);
      cmd.linear.x =
          std::min(kp_distance * error_distance * alignment_fact, max_linear);
    }
    cmd.angular.z = std::clamp(kp_yaw * error_yaw, -max_angular, max_angular);
    pub_->publish(cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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