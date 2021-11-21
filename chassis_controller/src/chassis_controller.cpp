#include "chassis_controller.hpp"

#define PI 3.1415926

// 控制器
ChassisController::ChassisController() : Node("mycontroller"){
    RCLCPP_INFO(this->get_logger(), "My chassis controller");

    // subscribe
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    odometry_sub.subscribe(this, "/simulator/odometry");
    canbus_sub.subscribe(this, "/simulator/canbus");
    gps_sub.subscribe(this, "/simulator/nav/gps");
    sync -> registerCallback(boost::bind(&ChassisController::msg_subscriber, this, _1, _2, _3, _4));

    // publish
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);
    timer = this->create_wall_timer(500ms, std::bind(&ChassisController::lateral_controller, this));
    
}

ChassisController::~ChassisController()
{
    delete sync;
}

void ChassisController::msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg)
{
    /*RCLCPP_INFO(this->get_logger(), "Accel X,Y: %.3f,%.3f [m/s^2] - Yaw rate: %.3f [deg/sec]",
              imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,imu_msg->angular_velocity.z);*/
    //传递变量
    vx = canbus_msg->speed_mps;
    vy = 0;
    phi = 0;
    x = gps_msg->pose.pose.position.x;
    y = gps_msg->pose.pose.position.y;
    delta = odometry_msg->front_wheel_angle;
    acc_x = imu_msg->linear_acceleration.x;
    acc_y = imu_msg->linear_acceleration.y;
    phi_p = imu_msg->angular_velocity.z;
    phi_p = phi_p*PI/180;
    RCLCPP_INFO(this->get_logger(), "Velocity: %f [m/s]", vx);
    RCLCPP_INFO(this->get_logger(), "Front wheel angle: %f [rad]", odometry_msg->front_wheel_angle);
    RCLCPP_INFO(this->get_logger(), "PositionX: %f [m]", x);
    RCLCPP_INFO(this->get_logger(), "PositionY: %f [m]", y);
}

void ChassisController::lateral_controller(){
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //前进档位
    control.acceleration_pct = 0.3;  //加速踏板
    control.braking_pct = 0; //制动踏板
    control.target_wheel_angle = 0.5; //车轮转角 rad
    control.target_wheel_angular_rate = 0; //车轮转角角速度 rad/s

    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; //自动驾驶模式激活
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;   //驾驶模式

    state_pub->publish(state);
    control_pub->publish(control);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisController>());
  rclcpp::shutdown();
  return 0;
}
