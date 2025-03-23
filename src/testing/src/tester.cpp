// pump_test_action_server.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include your custom action header. Adjust the package name if necessary.
#include "interfaces/action/pump_test.hpp"

using namespace std::chrono_literals;

class PumpTestActionServer : public rclcpp::Node
{
public:
  using PumpTest = interfaces::action::PumpTest;
  using GoalHandlePumpTest = rclcpp_action::ServerGoalHandle<PumpTest>;

  explicit PumpTestActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tester", options)
  {
    // Create the action server for PumpTest on the "/pump_test" topic.
    action_server_ = rclcpp_action::create_server<PumpTest>(
      this,
      "/pump_test",
      std::bind(&PumpTestActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PumpTestActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PumpTestActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PumpTest action server started");
  }

private:
  rclcpp_action::Server<PumpTest>::SharedPtr action_server_;

  // Called when a new goal request is received.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const PumpTest::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: pump_power_percent=%.2f, fuel_ml=%u",
                goal->pump_power_percent, goal->fuel_ml);
    // Accept all goals for this example.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Called when a cancel request is received.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePumpTest> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Called once a goal has been accepted. Start execution in a separate thread.
  void handle_accepted(const std::shared_ptr<GoalHandlePumpTest> goal_handle)
  {
    // This runs the execution callback in a new thread.
    std::thread{std::bind(&PumpTestActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execution callback: simulate the pump test.
  void execute(const std::shared_ptr<GoalHandlePumpTest> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing PumpTest action...");
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<PumpTest::Feedback>();
    auto result = std::make_shared<PumpTest::Result>();

    uint32_t fuel_pumped = 0;
    // Dummy pump rate calculation: 50% of pump_power_percent
    float pump_rate = goal->pump_power_percent * 0.5f;

    rclcpp::Rate loop_rate(1); // 1 Hz

    // Simulate pumping until the requested fuel_ml is reached.
    while (fuel_pumped < goal->fuel_ml) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "PumpTest goal canceled");
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      // Simulate pumping: increase fuel_pumped by 10 ml each iteration.
      fuel_pumped += 10;
      feedback->fuel_pumped_ml = fuel_pumped;
      feedback->fuel_pump_rate = pump_rate;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Pumped %u ml of fuel", fuel_pumped);
      loop_rate.sleep();
    }

    // Once complete, mark the result as successful.
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "PumpTest action succeeded");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PumpTestActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
