#include <bno055_driver.h>
#include "lifecycle_interface.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class BNO055Node : public Device {
public:
  BNO055Node(int addr) : Device("bno055_node") {
    bno055_ = std::make_shared<BNO055>(1, addr, 1);
    std::shared_ptr<std_srvs::srv::Trigger::Request> req;
    std::shared_ptr<std_srvs::srv::Trigger::Response> res;

    iservice_ = this->create_service<std_srvs::srv::Trigger>("bno055_init", std::bind(&BNO055Node::initSrvs, this, _1, _2));
  }

  bool initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    
    uint8_t ss, str, se;
    
    
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
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<BNO055Node>(BNO055_ADDRESS_A);

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
