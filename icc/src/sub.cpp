#include <chrono>
#include <memory>
#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/compressed_image.hpp"     
#include "sensor_msgs/msg/imu.hpp" 
#include "lgsvl_msgs/msg/detection3_d_array.hpp"
#include "lgsvl_msgs/msg/signal_array.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"


using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::Imu, lgsvl_msgs::msg::Detection3DArray, 
                                                          lgsvl_msgs::msg::SignalArray, lgsvl_msgs::msg::CanBusData> MySyncPolicy;


class msgSub : public rclcpp::Node
{
  public:
    msgSub();
    ~msgSub();

  private:

    // subscriber callback function declare
    void subscriber_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                              const lgsvl_msgs::msg::Detection3DArray::ConstSharedPtr& groundturth_msg, const lgsvl_msgs::msg::SignalArray::ConstSharedPtr& signal_msg, 
                              const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg);

    // subscribe object
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_sub;           // camera
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;                         // imu
    message_filters::Subscriber<lgsvl_msgs::msg::Detection3DArray> groundturth_sub;     // 3d ground truth
    message_filters::Subscriber<lgsvl_msgs::msg::SignalArray> signal_sub;               // light signal
    message_filters::Subscriber<lgsvl_msgs::msg::CanBusData> canbus_sub;                // canbus
    // time sync
    message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), image_sub, imu_sub, groundturth_sub, signal_sub, canbus_sub);
    // time set
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

msgSub::msgSub() : Node("msg_subscribe"), count_(0)
{
    // init info
    RCLCPP_INFO(this->get_logger(), "Init message subscribe node");

    // subscribe topic same with lgsvl sensor topic
    image_sub.subscribe(this, "/simulator/sensor/camera/center/image/compressed");
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    groundturth_sub.subscribe(this, "/simulator/ground_truth/m3d_detections");
    signal_sub.subscribe(this, "/simulator/ground_truth/signals");
    canbus_sub.subscribe(this, "/simulator/canbus");

    // register callback function
    sync -> registerCallback(boost::bind(&msgSub::subscriber_callback, this, _1, _2, _3, _4, _5));
}

msgSub::~msgSub()
{
    delete sync;
}

// subscriber callback function definition
void msgSub::subscriber_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                                    const lgsvl_msgs::msg::Detection3DArray::ConstSharedPtr& groundturth_msg, const lgsvl_msgs::msg::SignalArray::ConstSharedPtr& signal_msg, 
                                    const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg)
{
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
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msgSub>());
  rclcpp::shutdown();
  return 0;
}