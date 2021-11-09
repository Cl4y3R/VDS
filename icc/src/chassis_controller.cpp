// 控制器
class chassisController
{
  public:
    chassisController(){};
    ~chassisController(){};
    double long_controller(double long_acc);
};

double chassisController::long_controller(double long_acc){
  double long_out = long_acc*long_acc;
  return long_out;
}