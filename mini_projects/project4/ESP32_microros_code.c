#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

const int potPin = 34;

rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_support_t support;
rcl_allocator_t allocator;

// ===================== Publisher Task =====================
void potentiometer_publisher(void *arg)
{
    (void)arg;

    while (1)
    {
        msg.data = (analogRead(potPin) * 255) / 4095;
        rcl_publish(&publisher, &msg, NULL);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Task every 100ms
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    set_microros_transports();

    allocator = rcl_get_default_allocator();

    // Init support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create Node
    rclc_node_init_default(&node, "esp32_pot_publisher", "", &support);

    // Create Publisher (correct message type!!)
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "potentiometer"
    );

    // Create publishing task
    xTaskCreatePinnedToCore(
        potentiometer_publisher,
        "potentiometer_publisher",
        5000,
        NULL,
        1,
        NULL,
        1
    );
}

void loop()
{
    // nothing here
}

