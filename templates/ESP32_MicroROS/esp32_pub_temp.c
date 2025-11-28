#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// ---------- micro-ROS Globals ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Int32 msg;

TaskHandle_t publisher_task_handle;

// ---------- Publisher Task ----------
void publisher_task(void *pvParameters)
{
    while (1)
    {
        msg.data++;                                // Update message
        rcl_publish(&publisher, &msg, NULL);        // Publish
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    set_microros_transports();

    delay(2000);

    // ----- micro-ROS Init (all in setup) -----
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(
        &node,
        "esp32_pub_node",
        "",
        &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32_counter");

    // Executor is required even if not used / used for the timer so delete it
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // ----- Create FreeRTOS Task -----
    xTaskCreatePinnedToCore(
        publisher_task,
        "publisher_task",
        4096,
        NULL,
        1,
        &publisher_task_handle,
        1);
}

void loop()
{
    // Nothing here
}

