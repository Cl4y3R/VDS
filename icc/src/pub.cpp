#include <chrono>
#include <memory>
#include <boost/bind.hpp>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

// 控制器
#include "chassis_controller.cpp"


using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::Imu, lgsvl_msgs::msg::Detection3DArray, 
                                                          lgsvl_msgs::msg::SignalArray, lgsvl_msgs::msg::CanBusData> MySyncPolicy;


//需要继承控制器类！
class msgPub : public rclcpp::Node, public chassisController
{
  public:
    msgPub();
    ~msgPub();

    void publisher_callback();

    // 松时间同步
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), image_sub, imu_sub, groundturth_sub, signal_sub, canbus_sub);

    // 消息发布对象
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;           // vehicle state
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;      // vehicle control
    // 发布定时
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

//类外定义构造函数，并初始化成员列表
msgPub::msgPub() : Node("msg_publish"), count_(0)
{
    // INFO
    RCLCPP_INFO(this->get_logger(), "Init message publish node");
    
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&msgSubPub::publisher_callback, this));    // 定时执行发布的回调函数
}

msgPub::~msgPub()
{
    delete sync;
}

// 发布者的回调函数
void msgPub::publisher_callback()
{
    /*车辆控制
      float32 acceleration_pct  # 0 to 1
      float32 braking_pct  # 0 to 1
      float32 target_wheel_angle  # radians
      float32 target_wheel_angular_rate  # radians / second
      uint8 target_gear

      uint8 GEAR_NEUTRAL = 0
      uint8 GEAR_DRIVE = 1
      uint8 GEAR_REVERSE = 2
      uint8 GEAR_PARKING = 3
      uint8 GEAR_LOW = 4*/
    //auto my_out = chassisController::long_controller(imu_msg->linear_acceleration.x);
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //前进档位
    control.acceleration_pct = 0;  //加速踏板
    control.braking_pct = 0; //制动踏板
    control.target_wheel_angle = 0; //车轮转角
    control.target_wheel_angular_rate = 0.1; //车轮转角角速度

    /*车辆状态
      uint8 blinker_state
      uint8 headlight_state
      uint8 wiper_state
      uint8 current_gear
      uint8 vehicle_mode
      bool hand_brake_active
      bool horn_active
      bool autonomous_mode_active

      uint8 BLINKERS_OFF = 0
      uint8 BLINKERS_LEFT = 1
      uint8 BLINKERS_RIGHT = 2
      uint8 BLINKERS_HAZARD = 3

      uint8 HEADLIGHTS_OFF = 0
      uint8 HEADLIGHTS_LOW = 1
      uint8 HEADLIGHTS_HIGH = 2

      uint8 WIPERS_OFF = 0
      uint8 WIPERS_LOW = 1
      uint8 WIPERS_MED = 2
      uint8 WIPERS_HIGH = 3

      uint8 GEAR_NEUTRAL = 0
      uint8 GEAR_DRIVE = 1
      uint8 GEAR_REVERSE = 2
      uint8 GEAR_PARKING = 3
      uint8 GEAR_LOW = 4

      uint8 VEHICLE_MODE_COMPLETE_MANUAL = 0
      uint8 VEHICLE_MODE_COMPLETE_AUTO_DRIVE = 1
      uint8 VEHICLE_MODE_AUTO_STEER_ONLY = 2
      uint8 VEHICLE_MODE_AUTO_SPEED_ONLY = 3
      uint8 VEHICLE_MODE_EMERGENCY_MODE = 4*/
    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; //自动驾驶模式激活
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;   //驾驶模式

    state_pub->publish(state);
    control_pub->publish(control);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count_++);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msgPub>());
  rclcpp::shutdown();
  return 0;
}