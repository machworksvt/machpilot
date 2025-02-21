#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <sys/sysinfo.h>

using namespace std::chrono_literals;

class JetsonStatusMonitor : public rclcpp::Node {
public:
    JetsonStatusMonitor() : Node("resource_monitor") {
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("jetson_temperature", 10);
        power_pub_ = this->create_publisher<std_msgs::msg::Float32>("jetson_power_usage", 10);
        cpu_usage_pub_ = this->create_publisher<std_msgs::msg::Float32>("jetson_cpu_usage", 10);
        memory_usage_pub_ = this->create_publisher<std_msgs::msg::Float32>("jetson_memory_usage", 10);
        gpu_usage_pub_ = this->create_publisher<std_msgs::msg::Float32>("jetson_gpu_usage", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("jetson_status", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&JetsonStatusMonitor::publish_status, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_usage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr memory_usage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gpu_usage_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float get_jetson_temperature() {
        std::ifstream file("/sys/devices/virtual/thermal/thermal_zone0/temp");
        float temp = 0.0;
        if (file.is_open()) {
            file >> temp;
            temp /= 1000.0;
        }
        return temp;
    }

    float get_power_usage() {
        std::ifstream file("/sys/bus/i2c/drivers/ina3221x/6-0040/iio_device/in_power0_input");
        float power = 0.0;
        if (file.is_open()) {
            file >> power;
            power /= 1000.0;
        }
        return power;
    }

    float get_cpu_usage() {
        struct sysinfo info;
        sysinfo(&info);
        return (float)info.loads[0];
    }

    float get_memory_usage() {
        struct sysinfo info;
        sysinfo(&info);
        return (1.0 - ((float)info.freeram / (float)info.totalram)) * 100.0;
    }

    float get_gpu_usage() {
        std::string cmd = "tegrastats --interval 100 --logfile /tmp/tegrastats.log & sleep 0.2; pkill tegrastats";
        std::system(cmd.c_str());
        std::ifstream file("/tmp/tegrastats.log");
        std::string line;
        if (std::getline(file, line)) {
            auto pos = line.find("GR3D_FREQ=");
            if (pos != std::string::npos) {
                return std::stof(line.substr(pos + 10, line.find('%') - pos - 10));
            }
        }
        return 0.0;
    }

    void publish_status() {
        auto temp = get_jetson_temperature();
        auto power = get_power_usage();
        auto cpu_usage = get_cpu_usage();
        auto memory_usage = get_memory_usage();
        auto gpu_usage = get_gpu_usage();

        sensor_msgs::msg::Temperature temp_msg;
        temp_msg.temperature = temp;
        temp_pub_->publish(temp_msg);

        std_msgs::msg::Float32 power_msg;
        power_msg.data = power;
        power_pub_->publish(power_msg);

        std_msgs::msg::Float32 cpu_msg;
        cpu_msg.data = cpu_usage;
        cpu_usage_pub_->publish(cpu_msg);

        std_msgs::msg::Float32 memory_msg;
        memory_msg.data = memory_usage;
        memory_usage_pub_->publish(memory_msg);

        std_msgs::msg::Float32 gpu_msg;
        gpu_msg.data = gpu_usage;
        gpu_usage_pub_->publish(gpu_msg);

        std_msgs::msg::String status_msg;
        status_msg.data = "Temp: " + std::to_string(temp) + " C, Power: " + std::to_string(power) + " W, CPU: " + std::to_string(cpu_usage) + "%, Mem: " + std::to_string(memory_usage) + "%, GPU: " + std::to_string(gpu_usage) + "%";
        status_pub_->publish(status_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetsonStatusMonitor>());
    rclcpp::shutdown();
    return 0;
}
