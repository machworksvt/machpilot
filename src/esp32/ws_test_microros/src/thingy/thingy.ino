#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/empty.h>

// ROS handles
rcl_subscription_t heartbeat_sub;
std_msgs__msg__Empty msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

// Timing
unsigned long last_heartbeat = 0;
const unsigned long TIMEOUT_MS = 5000; // 5 seconds until we consider it disconnected

// Callback when a heartbeat message is received
void heartbeat_callback(const void * msgin) {
  (void) msgin;
  last_heartbeat = millis();
  Serial.println("Heartbeat received!");
}

// Setup node
void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("\nBooting at 115200 baud...");

  set_microros_transports();
  delay(2000);
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "esp32_heartbeat_listener", "", &support);

  // Create subscription to heartbeat
  rclc_subscription_init_default(
    &heartbeat_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/heartbeat");

  // Initialize executor with one handle (the subscriber)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &heartbeat_sub, &msg, &heartbeat_callback, ON_NEW_DATA);

  last_heartbeat = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("ESP32 heartbeat listener started!");
}

void loop() {

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  unsigned long now = millis();

  if (now - last_heartbeat > TIMEOUT_MS) {
    //Heartbeat missing
    Serial.println("Lost connection! Triggering...");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
