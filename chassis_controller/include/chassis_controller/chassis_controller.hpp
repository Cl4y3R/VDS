#include <chrono>
#include <cmath>
#include <memory>
#include <boost/bind.hpp>
// 消息过滤与时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// 自定义类继承该基类
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 消息类型-订阅   
#include "sensor_msgs/msg/imu.hpp" 
#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
// 消息类型-发布
#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, lgsvl_msgs::msg::VehicleOdometry, 
                                                        lgsvl_msgs::msg::CanBusData, nav_msgs::msg::Odometry> MySyncPolicy;


class ChassisController: public rclcpp::Node{
    public:
        ChassisController();
        ~ChassisController();
    
    private:
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<lgsvl_msgs::msg::VehicleOdometry> odometry_sub;
        message_filters::Subscriber<lgsvl_msgs::msg::CanBusData> canbus_sub;
        message_filters::Subscriber<nav_msgs::msg::Odometry> gps_sub;
        
        //msgs sync
        message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), imu_sub, odometry_sub, canbus_sub, gps_sub);

        rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;

        //timer
        rclcpp::TimerBase::SharedPtr timer;
        
        //variables
        double vx;
        double vy;
        double phi;
        double phi_p;
        double x;
        double y;
        double delta;
        double acc_x;
        double acc_y;
        
        //subscribed msgs
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
        lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr odometry_msg;
        lgsvl_msgs::msg::CanBusData::ConstSharedPtr canbus_msg;
        nav_msgs::msg::Odometry::ConstSharedPtr gps_msg;

        //回调函数
        void msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg);
        void lateral_controller();
        double steer_controller(double yaw, double yaw_rate, double pos_x, double pos_y, double velocity_x, double velocity_y);
};