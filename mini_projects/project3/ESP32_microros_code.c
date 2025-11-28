#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define BUTTON_PIN 22

// micro-ROS objects
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Bool msg;

// RTOS task handle
TaskHandle_t button_task_handle;

// =============================
// RTOS TASK (publishes button)
// =============================
void button_task(void *arg)
{
    while (true)
    {
        msg.data = (digitalRead(BUTTON_PIN) == LOW);  
        rcl_publish(&publisher, &msg, NULL);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        vTaskDelay(pdMS_TO_TICKS(50));   // 50ms polling
    }
}

// =============================
// SETUP
// =============================
void setup()
{
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    set_microros_transports();

    // Init micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Node
    rclc_node_init_default(
        &node,
        "esp32_button_publisher",
        "",
        &support
    );

    // Publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "button"
    );

    rclc_executor_init(&executor, &support.context, 0, &allocator);

    msg.data = false;

    // ============================
    // CREATE RTOS TASK
    // ============================
    xTaskCreatePinnedToCore(
        button_task,
        "button_task",
        4000,
        NULL,
        1,
        &button_task_handle,
        1   
    );

}

void loop()
{
    // Nothing here — RTOS handles everything
}

