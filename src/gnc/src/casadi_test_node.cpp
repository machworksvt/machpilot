#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>

using namespace casadi;

int main(int argc, char * argv[])
{
  // 1) Init ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("casadi_test_node");

  // 2) Build the symbolic function f(x) = x^2 + 2*x + 1
  casadi::SX x    = casadi::SX::sym("x");
  casadi::SX expr = x*x + 2*x + 1;
  auto f = casadi::Function("f", {x}, {expr});

  // 3) Create a DM holding the numeric argument 3.0
  casadi::DM arg = casadi::DM(3.0);

  // 4) Call the Function via its DM overload and grab the first (only) output
  std::vector<casadi::DM> out = f(arg);
  double v = static_cast<double>(out[0].scalar());

  // 5) Log the result
  RCLCPP_INFO(node->get_logger(), "f(3) = %g (expected 16)", v);

  // 6) Clean up
  rclcpp::shutdown();
  return 0;
}
