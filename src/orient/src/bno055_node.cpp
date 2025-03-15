#include "BNO055.h"
#include "sensor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class BNO055Node : public Sensor {
public:
  BNO055Node(int adr) : Sensor("bno055_node") {
    bno055_ = std::make_shared<BNO055>(1, adr, 1);
    std::shared_ptr<std_srvs::srv::Trigger::Request> req;
    std::shared_ptr<std_srvs::srv::Trigger::Response> res;

    iservice_ = this->create_service<std_srvs::srv::Trigger>("bno055_init", std::bind(&BNO055Node::initSrvs, this, _1, _2));
  }

  bool initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    
    bno055_->begin(bno055_->OPERATION_MODE_IMUPLUS);

    uint8_t ss, str, se;
    bno055_->getSystemStatus(&ss, &str, &se);

    printf("BNO055 system status: %i", ss);
    printf("BNO055 self test results: %i", str);
    printf("BNO055 system error: %i", se);
    
    if (!ss && !str && !se) {
        res->set__success(true);
        return true;
    }
    res->set__success(false);
    return false;
  }

  void calibAct() {

  }

private:
  std::shared_ptr<BNO055> bno055_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr iservice_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO055Node>(0x28));
  rclcpp::shutdown();
  return 0;
}
