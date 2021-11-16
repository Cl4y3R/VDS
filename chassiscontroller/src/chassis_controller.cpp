#include "icc/chassis_controller.cpp"
// 控制器
ChassisController::ChassisController(){
    RCLCPP_INFO(this->get_logger(), "My chassis controller");

    // subscribe
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    groundturth_sub.subscribe(this, "/simulator/ground_truth/m3d_detections");
    canbus_sub.subscribe(this, "/simulator/canbus");

    // publish
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);

    std::bind(&ChassisController::long_controller, this, std::placeholders::_1));
}


void ChassisController::long_controller(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg){
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //前进档位
    control.acceleration_pct = imu_msg->linear_acceleration.x;  //加速踏板
    control.braking_pct = 0; //制动踏板
    control.target_wheel_angle = 0; //车轮转角
    control.target_wheel_angular_rate = 0; //车轮转角角速度

    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; //自动驾驶模式激活
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;   //驾驶模式

    state_pub->publish(state);
    control_pub->publish(control);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<ChassisController>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
