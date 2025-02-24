#include "controller.hpp"
#include "rc/math/math.h"
#include "rc/time.h"

class STATE_ESTMATORNode : public Controller
{
public:
    STATE_ESTMATORNode() : Controller("state_estimator_node") {

    }
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STATE_ESTMATORNode>());
  rclcpp::shutdown();
  return 0;
}