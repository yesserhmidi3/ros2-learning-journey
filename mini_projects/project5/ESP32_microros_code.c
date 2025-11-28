#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define LED_PIN 16

// ---------- micro-ROS Globals ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;

std_msgs__msg__Int32 msg;

TaskHandle_t led_task_handle;

// ---------- Callback ----------
void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *in_msg = (const std_msgs__msg__Int32 *)msgin;
    analogWrite(LED_PIN, in_msg->data );
}

// ---------- Subscriber Task ----------
void led_task(void *pvParameters)
{
    while (1)
    {
        rclc_executor_spin_some(&executor, 10);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    set_microros_transports();

    delay(2000);

    // ----- micro-ROS Init (all in setup) -----
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(
        &node,
        "led_dimmer",
        "",
        &support);

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "led");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &msg,
        &subscription_callback,
        ON_NEW_DATA);

    // ----- Create FreeRTOS Task -----
    xTaskCreatePinnedToCore(
        led_task,
        "led_task",
        4096,
        NULL,
        1,
        &led_task_handle,
        1);
}

void loop()
{
    // Nothing here
}

