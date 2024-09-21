#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <IBusBM.h>  // Добавляем библиотеку IBusBM

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

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
std_msgs__msg__Int32 msg_subscriber;

// Timer and executor for timed RC publisher
rcl_timer_t timer;
rclc_executor_t executor_pub_rc;
rclc_executor_t executor_sub;

IBusBM IBus; // IBus object для чтения данных

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
    delay(100);
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
  const std_msgs__msg__Int32 *msg_subscriber = (const std_msgs__msg__Int32 *)msgin;

  // Publish a fixed value for encoders
  msg_encoders.data = 313; // Example value
  RCSOFTCHECK(rcl_publish(&encoders_publisher, &msg_encoders, NULL));

  // Debug output to Serial
  Serial.print("Received cmd_velocities: ");
  Serial.println(" | Encoders published: 313");
}

/**
 * @brief Timer callback for RC publisher. Publishes RC data at intervals.
 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Чтение данных с канала 1 iBus
    int rc_value = IBus.readChannel(1);

    // Если данные с iBus корректные (не 0), публикуем их. Иначе публикуем 1
    if (rc_value != 0)
    {
      msg_rc.data = rc_value;
    }
    else
    {
      msg_rc.data = 1; // Если данных нет, публикуем 1
    }

    RCSOFTCHECK(rcl_publish(&rc_publisher, &msg_rc, NULL));

    // Debug output to Serial
    Serial.print("RC channel 1: ");
    Serial.println(msg_rc.data);
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

  // Инициализация iBus
  IBus.begin(Serial2, 1); // Подключаем iBus к Serial2 и используем таймер 1

  Serial.println("Start IBus2PWM_ESP32");

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

  // Initialize subscriber for cmd_velocities
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "cmd_velocities"));

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

  // Чтение данных iBus для вывода на монитор
  Serial.print(IBus.readChannel(1)); // Отображение значений канала 1
  Serial.print("   ");
  Serial.println(IBus.readChannel(2)); // Отображение значений канала 2

  delay(100);
}
