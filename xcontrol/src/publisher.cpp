#include "publisher.hpp"

// publisher
MsgPublisher::MsgPublisher() : Node("msg_publisher"){
    RCLCPP_INFO(this->get_logger(), "Start publishing to lgsvl");

    // subscribe
    msg_from_control = this->create_subscription<std_msgs::msg::String>(
        "control_message", 10, std::bind(&MsgPublisher::msg_from_control, this, _1));

    // publish
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);
    timer = this->create_wall_timer(500ms, std::bind(&MsgPublisher::publish_to_lgsvl, this));
    
}

void MsgPublisher::subscribe_from_control()
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void MsgPublisher::publish_to_lgsvl(){
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //gear
    control.acceleration_pct = 0;  //acc in percentage
    control.braking_pct = 0; //brake in percentage
    control.target_wheel_angle = 0; //steering angle in rad
    control.target_wheel_angular_rate = 0; //steering angle velocity in rad/s

    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; 
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;

    state_pub->publish(state);
    control_pub->publish(control);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgPublisher>());
  rclcpp::shutdown();
  return 0;
}