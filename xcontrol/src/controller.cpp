#include "controller.hpp"

// 控制器
Controller::Controller() : Node("controller"){
    RCLCPP_INFO(this->get_logger(), "Controller started to work");

    // subscribe
    msg_from_subscriber= this->create_subscription<std_msgs::msg::String>(
        "subscriber_message", 10, std::bind(&Controller::subscribe_from_subscriber, this, _1));
    //load waypoint
    waypoint=waypoint_loader("./maps/pointmap/point.csv");
    for (auto i: waypoint)
        cout<<"X: "<<i[0]<<" Y: "<<i[1]<<endl;

    // publish
    msg_to_publisher = this->create_publisher<std_msgs::msg::String>("control_message", 10);
    timer = this->create_wall_timer(500ms, std::bind(&Controller::publish_to_publisher, this));
    
}

Controller::~Controller()
{
    delete sync;
}

void Controller::subscribe_from_subscriber(){
    //
}

void Controller::publish_to_publisher(){
    //
}

vector<vector<double>> Controller::waypoint_loader(string filename){
    vector<vector<double>> waypoint_container;
    ifstream fin(filename);
    string line;
    while (getline(fin, line))
    {
        istringstream sin(line);
        vector<string> Waypoints;
        string info;
        vector<double> x_y;
        while(getline(sin, info, ',')){
            Waypoints.push_back(info);
        }
        string x_str=Waypoints[4];
        string y_str=Waypoints[5];
        double waypoint_x,waypoint_y;
        stringstream sx,sy;
        sx<<x_str;
        sy<<y_str;
        sx>>waypoint_x;
        sy>>waypoint_y;
        waypoint_x=-waypoint_x;
        x_y.push_back(waypoint_x);
        x_y.push_back(waypoint_y);
        waypoint_container.push_back(x_y);

        //waypoint smoother
        vector<vector<double>> new_waypoint;
    }
    if(waypoint_container.empty()) cout<<"File is empty!!!!"<<endl;
    cout<<"Waypoints are loaded!"<<endl;
    return waypoint_container;
}

double Controller::longitudinal_controller(double velocity_x, double acc_x){
    return 0; //need to create 2x1 vector which holds acc_pct and brake_pct
}

double Controller::lateral_controller(double yaw, double yaw_rate, double pos_x, double pos_y, double velocity_x){
    double target_steer_angle = 0;
    return target_steer_angle;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}