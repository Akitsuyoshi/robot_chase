#include "rclcpp/rclcpp.hpp"

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase_node") {
    RCLCPP_INFO(get_logger(), "Robot Chase Node is Ready");
  }

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}