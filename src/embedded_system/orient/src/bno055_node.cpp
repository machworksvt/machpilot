#include <bno055_driver.h>
#include "lifecycle_interface.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class BNO055Node : public Device {
public:
  
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
