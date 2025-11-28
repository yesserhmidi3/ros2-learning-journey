#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// micro-ROS objects
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Int32 msg;

int counter = 0;

// RTOS task handle
TaskHandle_t counter_task_handle;

// =============================
// RTOS TASK — publishes counter
// =============================
void counter_task(void *arg)
{
    while (true)
    {
        msg.data = counter;
        rcl_publish(&publisher, &msg, NULL);

        counter++;

        // allow micro-ROS background handling
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second interval
    }
}

// =============================
// SETUP
// =============================
void setup()
{
    Serial.begin(115200);
    set_microros_transports();

    counter = 0;

    // Step 1 — Initialize micro-ROS support
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Step 2 — Create node
    rclc_node_init_default(&node, "esp32_counter_node", "", &support);

    // Step 3 — Create publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "counter"
    );

    // Step 4 — Initialize executor (no timers)
    rclc_executor_init(&executor, &support.context, 0, &allocator);

    msg.data = 0;

    // ============================
    // Step 5 — Create RTOS task
    // ============================
    xTaskCreatePinnedToCore(
        counter_task,
        "counter_task",
        4000,
        NULL,
        1,
        &counter_task_handle,
        1   // run on core 1 (recommended for micro-ROS + WiFi transport)
    );

    Serial.println("Counter RTOS task started!");
}

void loop()
{
    // empty — RTOS handles everything
}

