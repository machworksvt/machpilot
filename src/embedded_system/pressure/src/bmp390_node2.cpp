#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "lifecycle_interface.hpp"

// Adjust these includes to your actual file names in the BMP390 driver
extern "C" {
  #include "driver_bmp390_basic.h"
}

#define ACCEPTABLE_T_LOW -10.0f
#define ACCEPTABLE_T_HIGH 70.0f

#define ACCEPTABLE_P_LOW 90000.0f
#define ACCEPTABLE_P_HIGH 110000.0f

class BMP390Node : public Device
{
public:
    BMP390Node(bmp390_address_t addr) : Device("bmp390_node")
    {
        addr_ = addr;
    }

private:
CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    // Initialize the sensor
    bmp390_interface_t interface = BMP390_INTERFACE_IIC;
    uint8_t status = bmp390_basic_init(interface, addr_);

    if (status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BMP390, error code: %d", status);
        return CallbackReturn::FAILURE;
    } 
    
    

    RCLCPP_INFO(this->get_logger(), "BMP390 located");

    initialized_ = true;

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (bmp390_basic_deinit()) {
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    float temperature_c = 0.0f;
    float pressure_pa   = 0.0f;
    int status = bmp390_basic_read(&temperature_c, &pressure_pa);
    
    if (status != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to read BMP390 data, error code: %d", status);
        return CallbackReturn::FAILURE;
    }

    if (temperature_c < ACCEPTABLE_T_HIGH && temperature_c > ACCEPTABLE_T_LOW) {
        RCLCPP_WARN(this->get_logger(), "BMP390 temperature data out of bounds, error code: 2");
        return CallbackReturn::FAILURE;
    }

    if (pressure_pa < ACCEPTABLE_P_HIGH && pressure_pa > ACCEPTABLE_P_LOW) {
        RCLCPP_WARN(this->get_logger(), "BMP390 pressure data out of bounds, error code: 3");
        return CallbackReturn::FAILURE;
    }

    // Create publishers for temperature and fluid pressure
    temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("bmp_temp", 10);
    pressure_publisher_    = this->create_publisher<sensor_msgs::msg::FluidPressure>("bmp_press", 10);

    // Create a timer to read and publish sensor data every second
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds((int)(DT * 1000)),
    std::bind(&BMP390Node::timer_callback, this)
    );

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (temperature_publisher_ != nullptr) {
        temperature_publisher_->~Publisher();
    }

    if (pressure_publisher_ != nullptr) {
        pressure_publisher_->~Publisher();
    }

    if (timer_ != nullptr) {
        timer_->cancel();
        timer_->~TimerBase();
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (temperature_publisher_ != nullptr) {
        temperature_publisher_->~Publisher();
    }

    if (pressure_publisher_ != nullptr) {
        pressure_publisher_->~Publisher();
    }

    timer_->cancel();
    timer_->~TimerBase();

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

int timer_callback() {

    float temperature_c = 0.0f;
    float pressure_pa   = 0.0f;
    uint8_t status = bmp390_basic_read(&temperature_c, &pressure_pa);

    if (status != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to read BMP390 data, error code: %d", status);
        return 1;
    }

    // Log the results
    RCLCPP_INFO(this->get_logger(),
                "BMP390: Temperature = %.2f °C, Pressure = %.2f Pa",
                temperature_c, pressure_pa);

    // Publish Temperature message
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = this->now();
    temp_msg.header.frame_id = "bmp390_frame";
    temp_msg.temperature = temperature_c;
    temp_msg.variance = 0.0;  // Unknown variance; adjust if available
    temperature_publisher_->publish(temp_msg);

    // Publish Fluid Pressure message
    auto press_msg = sensor_msgs::msg::FluidPressure();
    press_msg.header.stamp = this->now();
    press_msg.header.frame_id = "bmp390_frame";
    press_msg.fluid_pressure = pressure_pa;
    press_msg.variance = 0.0;  // Unknown variance; adjust if available
    pressure_publisher_->publish(press_msg);
    
    return 0;
}

    bool initialized_{false};
    rclcpp::TimerBase::SharedPtr timer_;

    bmp390_address_t addr_{BMP390_ADDRESS_ADO_LOW};

    bool timeout_flag_{true};

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<BMP390Node>(BMP390_ADDRESS_ADO_LOW);

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
