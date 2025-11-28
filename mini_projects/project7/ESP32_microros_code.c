#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define BUTTON_1 23
#define BUTTON_2 22

int x,y,x_pred,y_pred;

// ---------- micro-ROS Globals ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Int32MultiArray msg;

TaskHandle_t coordinates_task_handle;

// ---------- Publisher Task ----------
void coordinates_task(void *pvParameters)
{
    while (1)
    {
        x=(digitalRead(BUTTON_1) == LOW) ? 1 : 0;
        y=(digitalRead(BUTTON_2) == LOW) ? 1 : 0;
        if((x!=x_pred)||(y!=y_pred)){
            x_pred=x;
            y_pred=y;
            msg.data.data[0] = x;
            msg.data.data[1] = y;
            rcl_publish(&publisher, &msg, NULL);//publish
        }                               
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    Serial.begin(115200);
    set_microros_transports();

    // Allocate array memory
    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.size = 2;
    msg.data.capacity = 2;
    msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));// two integers

    x_pred=0;
    y_pred=0;
    

    delay(2000);

    // ----- micro-ROS Init (all in setup) -----
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(
        &node,
        "coordinates_publisher",
        "",
        &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "coordinates");
    // Executor is required even if not used / used for the timer 
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // ----- Create FreeRTOS Task -----
    xTaskCreatePinnedToCore(
        coordinates_task,
        "coordinates_task",
        4096,
        NULL,
        1,
        &coordinates_task_handle,
        1);
}

void loop()
{
    // Nothing here
}

