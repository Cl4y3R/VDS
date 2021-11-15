#include "icc/chassis_controller.hpp"
#include "chassis_controller.cpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msgSub>());
  rclcpp::spin(std::make_shared<msgPub>());
  rclcpp::shutdown();
  return 0;
}