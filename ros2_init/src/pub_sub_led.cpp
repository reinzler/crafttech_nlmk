#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#include <std_msgs/msg/int32.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_publisher;
rclc_executor_t executor_pub;

// Subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_subscriber;
rclc_executor_t executor_sub;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#define LED_PIN LED_BUILTIN


void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

/**
 * @brief Subscription callback executed at receiving a message
 * Here we publish a new message using the data received in the subscriber callback.
 *
 * @param msgin
 */
void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg_subscriber = (const std_msgs__msg__Int32 *)msgin;

  // Publish the received data incremented by 1
  msg_publisher.data = msg_subscriber->data + 1;
  RCSOFTCHECK(rcl_publish(&publisher, &msg_publisher, NULL));

  // Debug output to Serial
  Serial.print("Received: ");
  Serial.print(msg_subscriber->data);
  Serial.print(" | Published: ");
  Serial.println(msg_publisher.data);
}

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Это выключит диод после старта программы

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_xiao_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "xiao_heartbeat"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "xiao_led_state"));

  // Create executors
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));

  // Add subscriber to executor
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_subscriber, &subscription_callback, ON_NEW_DATA));

  // Initialize message data
  msg_publisher.data = 0;
}

void loop()
{
  // Process incoming subscriber messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));

  // Delay for a short period
  delay(100);
}
