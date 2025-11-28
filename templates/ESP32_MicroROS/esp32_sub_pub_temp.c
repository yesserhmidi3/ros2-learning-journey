#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 2
#define BUTTON_PIN 22

// --- ROS2 objects ---
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;

std_msgs__msg__Bool pub_msg;
std_msgs__msg__Bool sub_msg;

// ===================== Subscriber Callback =====================
void led_callback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    digitalWrite(LED_PIN, msg->data ? HIGH : LOW);
    Serial.print("[Subscriber] LED set to: ");
    Serial.println(msg->data);
}

// ===================== Publisher Task =====================
void publisher_task(void * arg)
{
    (void)arg;

    while(1)
    {
        // Read button (pressed = LOW, released = HIGH)
        pub_msg.data = digitalRead(BUTTON_PIN) == LOW;

        rcl_publish(&publisher, &pub_msg, NULL);
        Serial.print("[Publisher] Button state: ");
        Serial.println(pub_msg.data);

        vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms loop
    }
}

// ===================== Subscriber Task =====================
void subscriber_task(void * arg)
{
    (void)arg;

    while(1)
    {
        // Spin executor to handle subscriber messages
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    set_microros_transports();

    // --- micro-ROS support ---
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // --- Create node ---
    rclc_node_init_default(&node, "esp32_pubsub_node", "", &support);

    // --- Create publisher ---
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "button"
    );
    pub_msg.data = false;

    // --- Create subscriber ---
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "led_control"
    );

    // --- Executor ---
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &led_callback, ON_NEW_DATA);

    // --- Create FreeRTOS tasks ---
    xTaskCreatePinnedToCore(publisher_task, "publisher_task", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(subscriber_task, "subscriber_task", 10000, NULL, 1, NULL, 1);
}

void loop()
{
    // Nothing here; all handled by FreeRTOS tasks
}
