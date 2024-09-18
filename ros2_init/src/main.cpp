#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__String pub_msg;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * received_msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  // Преобразуем полученные данные в строку
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Received: %f, %f, %f", received_msg->data.data[0], received_msg->data.data[1], received_msg->data.data[2]);

  // Копируем строку в сообщение для публикации
  pub_msg.data.data = buffer;
  pub_msg.data.size = strlen(buffer);
  pub_msg.data.capacity = pub_msg.data.size + 1;

  // Публикуем сообщение
  RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "micro_ros_platformio_node_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_platformio_node_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

  // Initialize pub_msg
  pub_msg.data.data = (char *)malloc(1);
  pub_msg.data.size = 0;
  pub_msg.data.capacity = 1;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}