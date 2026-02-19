#include "lifecycle_interface.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "BNO055.h"
#include "imumaths.h"

#include <memory>
#include <iostream>
#include <math.h>

#define DEG_TO_RAD M_PI / 180.0

using std::placeholders::_1;
using std::placeholders::_2;

class BNO055Node : public Device
{
public:
BNO055Node(int addr) : Device("bno055_node")
{
    bno055_ = std::make_shared<BNO055>(BNO055_ID, addr, 7);
}

~BNO055Node()
{
    
}

private:
CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (bno055_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "BNO055: not initialized");
        return CallbackReturn::FAILURE;
    }
    
    if (!bno055_->begin(BNO055::OPERATION_MODE_NDOF)) {
        RCLCPP_ERROR(get_logger(), "BNO055: begin failed");
        return CallbackReturn::FAILURE;
    }

    // #TODO: calibration routine here maybe, maybe in on_activate

    uint8_t system = 0, self_test = 0, system_error = 0;

    while (system < 5 || self_test != 0x0F || system_error) {
        bno055_->getSystemStatus(&system, &self_test, &system_error);
        RCLCPP_WARN(get_logger(), "BNO055: system status not aligned");
        usleep(100);
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    bno055_.reset();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "bno_imu", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(DT * 1000.0)),
        std::bind(&BNO055Node::timer_callback, this));

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    timer_->cancel();
    timer_.reset();

    pub_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    timer_->cancel();
    timer_.reset();

    pub_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

int timer_callback() {

    auto msg = sensor_msgs::msg::Imu();
    imu::Vector<3> omega;
    imu::Vector<3> accel;
    imu::Vector<3>  grav;
    imu::Quaternion quat;

    omega = bno055_->getVector(BNO055::VECTOR_GYROSCOPE) * DEG_TO_RAD; // degrees per second
    accel = bno055_->getVector(BNO055::VECTOR_LINEARACCEL); // meters per second
    grav  = bno055_->getVector(BNO055::VECTOR_GRAVITY); // meters per second
    quat  = bno055_->getQuat(); // normalized quaternion
    
    msg.angular_velocity.set__x(omega[0]);
    msg.angular_velocity.set__y(omega[1]);
    msg.angular_velocity.set__z(omega[2]);
    
    msg.linear_acceleration.set__x(accel[0]);
    msg.linear_acceleration.set__y(accel[1]);
    msg.linear_acceleration.set__z(accel[2]);

    msg.orientation.set__w(quat.w());
    msg.orientation.set__x(quat.x());
    msg.orientation.set__y(quat.y());
    msg.orientation.set__z(quat.z());

    pub_->publish(msg);

    return 0;
    
}

private:
std::shared_ptr<BNO055> bno055_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

bool timeout_flag_{false};

};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<BNO055Node>(0x28);

  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
