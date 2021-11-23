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
    timer = this->create_wall_timer(500ms, std::bind(&ChassisController::control_publisher, this));
    
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
    //parameters
    vx = canbus_msg->speed_mps;
    vy = 0; //unused for now
    phi_p = imu_msg->angular_velocity.z;
    phi_p = phi_p*PI/180;
    phi = imu_msg->orientation.z;
    x = -gps_msg->pose.pose.position.y; //to be modified
    y = gps_msg->pose.pose.position.x; //to be modified
    delta = odometry_msg->front_wheel_angle;
    ax = imu_msg->linear_acceleration.x;
    ay = imu_msg->linear_acceleration.y;
    steer_angle = odometry_msg->front_wheel_angle;

    //RCLCPP_INFO(this->get_logger(), "Velocity: %f [m/s]", vx);
    //RCLCPP_INFO(this->get_logger(), "Front wheel angle: %f [rad]", steer_angle);
    RCLCPP_INFO(this->get_logger(), "PositionX: %f [m], PositionY: %f [m]", x, y);
}

void ChassisController::control_publisher(){
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

double ChassisController::longitudinal_controller(double velocity_x, double acc_x){
    return 0; //need to create 2x1 vector which holds acc_pct and brake_pct
}

double ChassisController::lateral_controller(double yaw, double yaw_rate, double pos_x, double pos_y, double velocity_x){
    double target_steer_angle = 0;
    return target_steer_angle;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisController>());
  rclcpp::shutdown();
  return 0;
}
