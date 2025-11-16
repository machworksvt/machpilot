#include <stdio.h>
#include <unistd.h>               // for sleep()
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/empty.h>

int main(int argc, const char * argv[])
{
    rcl_ret_t rc;

    // --- 1. Init options and context ---
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rc = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    if (rc != RCL_RET_OK) {
        fprintf(stderr, "Failed to init options: %s\n", rcl_get_error_string().str);
        return 1;
    }

    rcl_context_t context = rcl_get_zero_initialized_context();
    rc = rcl_init(argc, argv, &init_options, &context);
    if (rc != RCL_RET_OK) {
        fprintf(stderr, "Failed to init context: %s\n", rcl_get_error_string().str);
        return 1;
    }

    // --- 2. Create node ---
    rcl_node_t node = rcl_get_zero_initialized_node();
    const char * node_name = "heartbeat_publisher";
    const char * namespace_ = "";
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rc = rcl_node_init(&node, node_name, namespace_, &context, &node_ops);
    if (rc != RCL_RET_OK) {
        fprintf(stderr, "Failed to init node: %s\n", rcl_get_error_string().str);
        return 1;
    }

    // --- 3. Create publisher ---
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);

    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    rc = rcl_publisher_init(&publisher, &node, type_support, "/heartbeat", &pub_ops);
    if (rc != RCL_RET_OK) {
        fprintf(stderr, "Failed to init publisher: %s\n", rcl_get_error_string().str);
        return 1;
    }

    // --- 4. Publish periodically ---
    std_msgs__msg__Empty msg; // no fields to fill

    printf("Publishing heartbeat every 1 second...\n");
    while (rcl_context_is_valid(&context)) {
        rc = rcl_publish(&publisher, &msg, NULL);
        if (rc != RCL_RET_OK) {
            fprintf(stderr, "Publish failed: %s\n", rcl_get_error_string().str);
        } else {
            printf("Heartbeat sent\n");
        }
        sleep(1);
    }

    // --- 5. Cleanup ---
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rcl_shutdown(&context);

    return 0;
}
