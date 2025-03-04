#include "BNO055.h"
#include "sensor.hpp"

class BNO055Node : public Sensor {
public:
  BNO055Node(int adr) : Sensor("bno055_node") {
    bno055_ = std::make_shared<BNO055>(1, adr, 1);
    std::shared_ptr<std_srvs::srv::Trigger::Request> req;
    std::shared_ptr<std_srvs::srv::Trigger::Response> res;

    iservice_ = this->create_service<std_srvs::srv::Trigger>("bno055_init", initSrvs);
  }

  int initSrvs(std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    bno055_->begin(bno055_->OPERATION_MODE_IMUPLUS);

    uint8_t ss, str, se;
    bno055_->getSystemStatus(&ss, &str, &se);

    printf("BNO055 system status: %i", ss);
    printf("BNO055 self test results: %i", str);
    printf("BNO055 system error: %i", se);
    
    if (!ss && !str && !se) {
        res->set__success(true);
        return;
    }
    res->set__success(false);
  }
private:
  std::shared_ptr<BNO055> bno055_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr iservice_;
};
