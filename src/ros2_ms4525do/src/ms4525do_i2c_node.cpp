#include "ros2_ms4525do/ms4525do_i2c_driver.hpp"
#include <sensor.h>
#include <csignal>
#include <memory>

class MS4525DO : public Sensor {
public:
    MS4525DO(int address, std::string device, std::string name);
    bool initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool resetSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void run();
    void stop();
    bool publish();
private:
    std::unique_ptr<ms4525do::MS4525DOI2CDriver> pitot;

    std::string param_device;
    int param_address_read;
    int param_address_write;
    double param_rate;
    std::string param_frame_id;

    diagnostic_msgs::msg::DiagnosticStatus current_status;

    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_press;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset;

    std::unique_ptr<rclcpp::Rate> rate;

    int seq;

};

MS4525DO::MS4525DO(int address, std::string device, std::string name) : Sensor(name) {
    pitot = std::make_unique<ms4525do::MS4525DOI2CDriver>(device, address);

    this->declare_parameter<std::string>("device", pitot->device);
    this->declare_parameter<int>("address_read", pitot->address_read);
    this->declare_parameter<int>("address_write", pitot->address_write);
    this->declare_parameter<std::string>("frame_id", "pitot");
    this->declare_parameter<double>("rate", 1000.0);

    this->get_parameter("device", param_device);
    this->get_parameter("address_read", param_address_read);
    this->get_parameter("address_write", param_address_write);
    this->get_parameter("frame_id", param_frame_id);
    this->get_parameter("rate", param_rate);

    

    pub_press = this->create_publisher<sensor_msgs::msg::FluidPressure>(name + "_press", 10);
    pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>(name + "_temp", 10);
    pub_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(name + "_status", 10);
    
    srv_init = this->create_service<std_srvs::srv::Trigger>("init", std::bind(&MS4525DO::initSrvs, this, std::placeholders::_1, std::placeholders::_2));
    srv_reset = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&MS4525DO::resetSrvs, this, std::placeholders::_1, std::placeholders::_2));

    const std::shared_ptr<std_srvs::srv::Trigger::Request> req;
    std::shared_ptr<std_srvs::srv::Trigger::Response> res;

    initSrvs(req, res);

    seq = 0;

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);

    rate = std::make_unique<rclcpp::Rate>(param_rate);
};

void MS4525DO::run() {
    while (rclcpp::ok()) {
        rate->sleep();
        if (publish()) {

        }
    }
};

bool MS4525DO::publish() {
    ms4525do::PITOTData data;

    try {
        pitot->readDF4();
    } catch(const std::runtime_error& e) {
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    pitot->getData(data); // update data values
    rclcpp::Time time = this->now();

    sensor_msgs::msg::FluidPressure msg_press;
    msg_press.header.stamp = time;
    msg_press.header.frame_id = param_frame_id;

    msg_press.fluid_pressure = (data.press_raw - MINP * (double)0x3FFF) / ((MAXP - MINP) * (double)0x3FFF);

    sensor_msgs::msg::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;

    msg_temp.temperature = (data.temp_raw * 200.0) / (double)0x7FF - 50.0;

    diagnostic_msgs::msg::DiagnosticStatus msg_status;
    msg_status.set__message("Status: " + data.status);

    pub_press->publish(msg_press);
    pub_temp->publish(msg_temp);
    pub_status->publish(msg_status);
};

bool MS4525DO::initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if(!pitot->init()) {
        res->success = false;
        return false;
    }

    res->success = true;
    return true;
};