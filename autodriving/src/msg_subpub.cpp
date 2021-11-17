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


using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::Imu, lgsvl_msgs::msg::Detection3DArray, 
                                                          lgsvl_msgs::msg::SignalArray, lgsvl_msgs::msg::CanBusData> MySyncPolicy;

class chassisController
{
  public:
    double long_controller(double long_acc);
};

double chassisController::long_controller(double long_acc){
  double long_out = long_acc*long_acc;
  return long_out;
}

//需要继承控制器类！
class msgSubPub : public rclcpp::Node, public chassisController
{
  public:
    msgSubPub();
    ~msgSubPub();

    // 消息订阅对象
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_sub;           // 摄像机传感器
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;                         // IMU传感器
    message_filters::Subscriber<lgsvl_msgs::msg::Detection3DArray> groundturth_sub;     // 3D 地面真相传感器
    message_filters::Subscriber<lgsvl_msgs::msg::SignalArray> signal_sub;               // 信号灯传感器
    message_filters::Subscriber<lgsvl_msgs::msg::CanBusData> canbus_sub;                // 车辆底盘传感器

    // 消息订阅的回调函数
    void subscriber_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                              const lgsvl_msgs::msg::Detection3DArray::ConstSharedPtr& groundturth_msg, const lgsvl_msgs::msg::SignalArray::ConstSharedPtr& signal_msg, 
                              const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg);
    
    // 消息发布的回调函数
    void publisher_callback();
    
    // 时间同步
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), image_sub, imu_sub, groundturth_sub, signal_sub, canbus_sub);

    // 消息发布对象
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;           // 车辆状态
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;      // 车辆控制
    // 发布定时
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

//类外定义构造函数，并初始化成员列表
msgSubPub::msgSubPub() : Node("msg_publish_subscribe"), count_(0)
{
    // INFO
    RCLCPP_INFO(this->get_logger(), "Init message publish and subscribe node");

    // 订阅消息 ""中的内容必须和对应的传感器Topic相同。
    image_sub.subscribe(this, "/simulator/sensor/camera/center/image/compressed");
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    groundturth_sub.subscribe(this, "/simulator/ground_truth/m3d_detections");
    signal_sub.subscribe(this, "/simulator/ground_truth/signals");
    canbus_sub.subscribe(this, "/simulator/canbus");
    // 注册回调函数
    sync -> registerCallback(boost::bind(&msgSubPub::subscriber_callback, this, _1, _2, _3, _4, _5));

    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&msgSubPub::publisher_callback, this));    // 定时执行发布的回调函数
}

msgSubPub::~msgSubPub()
{
    delete sync;
}

// 订阅者的回调函数
void msgSubPub::subscriber_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                                    const lgsvl_msgs::msg::Detection3DArray::ConstSharedPtr& groundturth_msg, const lgsvl_msgs::msg::SignalArray::ConstSharedPtr& signal_msg, 
                                    const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg)
{
    //Subscribe info
    RCLCPP_INFO(this->get_logger(), "Subscribed: Get 3D_ground_truth & Imu & signal & can_bus_data Message");
    //Subscribe time stamp info
    RCLCPP_INFO(this->get_logger(), "scan stamp:%d - %d - %d - %d -%d", image_msg->header.stamp.sec, imu_msg->header.stamp.sec, groundturth_msg->header.stamp.sec, 
                                                                    signal_msg->header.stamp.sec, canbus_msg->header.stamp.sec);
    //Subscribe imu info
    RCLCPP_INFO(this->get_logger(), "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
              imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
              imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
    //Subscribe can bus info
    RCLCPP_INFO(this->get_logger(), "Speed: %.3f [m/s] - Throttle: %.3f [-] - Brake: %.3f [-] - Steer: %.3f [-]",
              canbus_msg->speed_mps, canbus_msg->throttle_pct, canbus_msg->brake_pct, canbus_msg->steer_pct);
    //测试自己的控制器代码
    //调用控制器类中的函数
    auto my_out = chassisController::long_controller(imu_msg->linear_acceleration.x);
    RCLCPP_INFO(this->get_logger(), "+++++My controller output is: %.3f+++++++", my_out);
}
// 发布者的回调函数
void msgSubPub::publisher_callback()
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
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //前进档位
    control.acceleration_pct = 0.1;  //加速踏板
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
  rclcpp::spin(std::make_shared<msgSubPub>());
  rclcpp::shutdown();
  return 0;
}