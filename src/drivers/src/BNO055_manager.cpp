#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Adafruit_BNO055.h>

/* How this node should function
      

*/

class BNO055Node : public rclcpp::Node {
public:
    BNO055Node() : Node("BNO055") {
        
        RCLCPP_INFO(this->get_logger(), "hello!");
        
    }

    ~BNO055Node() noexcept override = default;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO055Node>());
  rclcpp::shutdown();
  return 0;
}