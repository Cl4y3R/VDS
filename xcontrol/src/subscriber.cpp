#include "publisher.hpp"

// subscriber
MsgSubscriber::MsgSubscriber() : Node("msg_subscriber"){
    RCLCPP_INFO(this->get_logger(), "Start subscribing from lgsvl");

    // subscribe
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    odometry_sub.subscribe(this, "/simulator/odometry");
    canbus_sub.subscribe(this, "/simulator/canbus");
    gps_sub.subscribe(this, "/simulator/nav/gps");
    sync -> registerCallback(boost::bind(&ChassisController::msg_subscriber, this, _1, _2, _3, _4));

    // publish
    msg_to_control = this->create_publisher<std_msgs::msg::String>("subscriber_message", 10);
    timer = this->create_wall_timer(500ms, std::bind(&MsgSubscriber::publish_to_control, this));
    
}

MsgSubscriber::~MsgSubscriber()
{
    delete sync;
}

void MsgSubscriber::subscribe_from_lgsvl(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
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

void MsgSubscriber::publish_to_control(){
    auto message = std_msgs::msg::String();
      message.data = "Hello, world!";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      msg_to_control->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgSubscriber>());
  rclcpp::shutdown();
  return 0;
}