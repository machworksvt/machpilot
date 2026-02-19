#include <rclcpp/rclcpp.hpp>
#include "ublox_gps_driver/gps_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ublox::GpsNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    executor.spin();
    rclcpp::shutdown();
    return 0;

}