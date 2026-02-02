#include "adxl345_driver.h"
#include <cstdio>

#include "lifecycle_interface.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    // auto node = std::make_shared<BNO055Node>(0x28);

    // exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}