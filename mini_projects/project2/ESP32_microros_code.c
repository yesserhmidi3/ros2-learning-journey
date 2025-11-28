#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 2

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;

// RTOS task handle
TaskHandle_t subscriber_task_handle;

// =============================
// SUBSCRIPTION CALLBACK
// =============================
void callback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
}

// =============================
// RTOS TASK (spins executor)
// =============================
void subscriber_task(void *arg)
{
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));

        vTaskDelay(pdMS_TO_TICKS(20));  
    }
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    set_microros_transports();

    // Init micro-ROS
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // Node
    rclc_node_init_default(&node, "esp32_led_node", "", &support);

    // Subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "led_control"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &callback, ON_NEW_DATA);

    // ============================
    // CREATE RTOS TASK
    // ============================
    xTaskCreatePinnedToCore(
        subscriber_task,
        "subscriber_task",
        4000,
        NULL,
        1,
        &subscriber_task_handle,
        1  
    );

    Serial.println("Subscriber RTOS task started!");
}

void loop()
{
    // empty — RTOS handles everything
}

