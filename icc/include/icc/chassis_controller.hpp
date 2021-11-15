#include <chrono>
#include <memory>
#include <boost/bind.hpp>
// 消息过滤与时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// 自定义类继承该基类
#include "rclcpp/rclcpp.hpp" //rclcpp是ROS2的C++API
#include "std_msgs/msg/string.hpp"

// 消息类型-订阅
#include "sensor_msgs/msg/compressed_image.hpp"     
#include "sensor_msgs/msg/imu.hpp" 
#include "lgsvl_msgs/msg/detection3_d_array.hpp"
#include "lgsvl_msgs/msg/signal_array.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
// 消息类型-发布
#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

class chassis_controller{
    public:
        double long_controller(double target_vx);
};
