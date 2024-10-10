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
#include <std_msgs/msg/float32_multi_array.h>

// Node and communication handles
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Publishers and message types
rcl_publisher_t encoders_publisher; // Publisher triggered by subscriber callback
std_msgs__msg__Int32 msg_encoders;

rcl_publisher_t rc_publisher;       // Publisher triggered by timer
std_msgs__msg__Int32 msg_rc;

// Subscriber and message type
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg_subscriber; // Для подписки на Float32MultiArray

// Timer and executor for timed RC publisher
rcl_timer_t timer;
rclc_executor_t executor_pub_rc;
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

// Error handling loop
void error_loop()
{
  while (1)
  {
    delay(10);
  }
}

/**
 * @brief Subscription callback executed at receiving a message.
 * This publishes a message with encoders data.
 *
 * @param msgin Pointer to incoming message.
 */
void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *msg_subscriber = (const std_msgs__msg__Float32MultiArray *)msgin;

  // Берем первый элемент из полученного сообщения и преобразуем его в int
  if (msg_subscriber->data.size > 0) {
    msg_encoders.data = (int32_t)msg_subscriber->data.data[0];

    // Публикуем значение
    RCSOFTCHECK(rcl_publish(&encoders_publisher, &msg_encoders, NULL));

    // Отладочный вывод в Serial
    Serial.print("Received cmd_velocities: ");
    Serial.print(msg_subscriber->data.data[0]);
    Serial.print(" | Encoders published: ");
    Serial.println(msg_encoders.data);
  } else {
    Serial.println("Received cmd_velocities, but no data available.");
  }
}

/**
 * @brief Timer callback for RC publisher. Publishes RC data at intervals.
 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Publish a fixed value for RC input
    msg_rc.data = 123; // Example value
    RCSOFTCHECK(rcl_publish(&rc_publisher, &msg_rc, NULL));

    // Debug output to Serial
    Serial.println("RC published: 123");
  }
}

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Turn off the LED after startup

  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize allocator and support
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  // Initialize encoders publisher (triggered by subscriber)
  RCCHECK(rclc_publisher_init_default(
      &encoders_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "encoders_output"));

  // Initialize RC publisher (triggered by timer)
  RCCHECK(rclc_publisher_init_default(
      &rc_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "rc_output"));

  // Установка QoS-профиля для подписчика
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  // Initialize subscriber for cmd_velocities
  RCCHECK(rclc_subscription_init(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "cmd_velocities",
      &qos_profile));

  // Инициализация данных сообщения для подписчика (инициализируем массив)
  msg_subscriber.data.data = (float *)malloc(sizeof(float) * 4); // Инициализируем массив на 4 элемента
  msg_subscriber.data.size = 4;
  msg_subscriber.data.capacity = 4;

  // Initialize timer for RC publisher, with a period of 1000 ms
  const unsigned int timer_timeout = 1000; // 1 second interval
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // Create executors
  RCCHECK(rclc_executor_init(&executor_pub_rc, &support.context, 2, &allocator)); // Handles timer
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));    // Handles subscriber

  // Add subscriber callback to executor
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_subscriber, &subscription_callback, ON_NEW_DATA));

  // Add timer callback to executor
  RCCHECK(rclc_executor_add_timer(&executor_pub_rc, &timer));

  // Initialize message data for both publishers
  msg_encoders.data = 0;
  msg_rc.data = 0;
}

void loop()
{
  // Spin the subscriber executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));

  // Spin the RC publisher executor (triggered by timer)
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub_rc, RCL_MS_TO_NS(100)));

  delay(100);
}
