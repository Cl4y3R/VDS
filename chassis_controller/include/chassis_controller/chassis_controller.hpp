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
// 消息类型-发布
#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, lgsvl_msgs::msg::VehicleOdometry, lgsvl_msgs::msg::CanBusData> MySyncPolicy;

class ChassisController: public rclcpp::Node{
    public:
        ChassisController();
        ~ChassisController();
    
    private:
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;                         // IMU传感器
        message_filters::Subscriber<lgsvl_msgs::msg::VehicleOdometry> odometry_sub;     // 里程器
        message_filters::Subscriber<lgsvl_msgs::msg::CanBusData> canbus_sub;                // 车辆底盘传感器
        
        //消息同步
        message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), imu_sub, odometry_sub, canbus_sub);

        rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;           // 车辆状态
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;      // 车辆控制

        //计时器
        rclcpp::TimerBase::SharedPtr timer;
        
        //中间变量参数
        double vx;
        double delta;
        double acc_x;
        double acc_y;
        double yaw_rate;

        //订阅消息
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
        lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr odometry_msg;
        lgsvl_msgs::msg::CanBusData::ConstSharedPtr canbus_msg;

        //回调函数
        void msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg);
        void controller();
};
