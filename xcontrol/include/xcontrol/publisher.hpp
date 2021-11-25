#include <chrono>
#include <cmath>
#include <memory>
#include <boost/bind.hpp>


// 自定义类继承该基类
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 消息类型-发布
#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

class MsgPublisher: public rclcpp::Node("msg_publisher"){
    public:
        MsgPublisher();
        ~MsgPublisher();
    
    private:
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        //timer
        rclcpp::TimerBase::SharedPtr timer;
        
        //subscribed msgs

        //ros2 functions
        void subscribe_from_control();
        void publish_to_lgsvl();
};