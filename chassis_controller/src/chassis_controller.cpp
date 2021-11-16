#include "chassis_controller.hpp"
// 控制器
ChassisController::ChassisController() : Node("mycontroller"), count(0){
    RCLCPP_INFO(this->get_logger(), "My chassis controller");

    // subscribe
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    groundturth_sub.subscribe(this, "/simulator/ground_truth/m3d_detections");
    canbus_sub.subscribe(this, "/simulator/canbus");
    sync -> registerCallback(boost::bind(&ChassisController::msg_subscriber, this, _1, _2, _3));
    
    // publish
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);
    timer = this->create_wall_timer(500ms, std::bind(&ChassisController::long_controller, this));
    
}

ChassisController::~ChassisController()
{
    delete sync;
}

void ChassisController::msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::Detection3DArray::ConstSharedPtr& groundturth_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg)
{
    RCLCPP_INFO(this->get_logger(), "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
              imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
              imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
}

void ChassisController::long_controller(){
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count++);
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //前进档位
    control.acceleration_pct = 1;  //加速踏板
    control.braking_pct = 0; //制动踏板
    control.target_wheel_angle = 0; //车轮转角
    control.target_wheel_angular_rate = 0; //车轮转角角速度

    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; //自动驾驶模式激活
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;   //驾驶模式

    state_pub->publish(state);
    control_pub->publish(control);
}

/*int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<ChassisController>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisController>());
  rclcpp::shutdown();
  return 0;
}
