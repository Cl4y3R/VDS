#include <chrono>
#include <cmath>
#include <memory>
#include <boost/bind.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
// 消息过滤与时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// 自定义类继承该基类
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PI 3.1415926

using namespace std::chrono_literals;
using std::string;
using std::vector;
using std::ifstream;
using std::istringstream;
using std::stringstream;
using std::cout;
using std::endl;

class Controller: public rclcpp::Node{
    public:
        Controller();
        ~Controller();
    
    private:

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
        double ax;
        double ay;
        double steer_angle;
        vector<vector<double>> waypoint;
        
        
        //waypoint load function
        vector<vector<double>> waypoint_loader(std::string filename);

        //controller functions
        double lateral_controller(double yaw, double yaw_rate, double pos_x, double pos_y, double velocity_x);
        double longitudinal_controller(double velocity, double acc_x);

        //ros2 functions
        void subscribe_from_subscriber();
        void publish_to_publisher();
};