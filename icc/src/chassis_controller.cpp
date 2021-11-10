#include "sub.cpp"
// 控制器
class chassisController:public msgSub
{
  public:
    chassisController();
    ~chassisController();
    double long_controller(msgSub::imu_sub);
};

double chassisController::long_controller(msgSub::imu_sub){
  double long_out = imu_msg->linear_acceleration.x*imu_msg->linear_acceleration.x;
  return long_out;
}
