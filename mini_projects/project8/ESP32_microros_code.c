#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

const int trigPin = 5;
const int echoPin = 18;
#define LED_1 2
#define LED_2 4

bool led_state_prev = false;

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distance;

// --- ROS2 objects ---
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;

std_msgs__msg__Float32 dist_msg;
std_msgs__msg__Bool led_msg;

// ===================== Subscriber Callback =====================
void led_callback(const void * msgin)
{
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if(msg->data != led_state_prev){
        digitalWrite(LED_1, msg->data ? HIGH : LOW);
        digitalWrite(LED_2, msg->data ? LOW  : HIGH);
        led_state_prev = msg->data;
    }
}

// ===================== Publisher Task =====================
void distance_publisher_task(void * arg)
{
    (void)arg;

    while(1)
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
        if (duration == 0) distance = -1; // out of range
        else distance = duration * SOUND_SPEED / 2;

        dist_msg.data=distance;                              
        rcl_publish(&publisher, &dist_msg, NULL); 

        vTaskDelay(250 / portTICK_PERIOD_MS); // 100ms loop
    }
}

// ===================== Subscriber Task =====================
void led_task(void * arg)
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
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);

    set_microros_transports();
    delay(1000);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "distance"
    );
    // --- Create subscriber ---
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "led"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &led_msg, &led_callback, ON_NEW_DATA);


    // --- Create FreeRTOS tasks ---
    xTaskCreatePinnedToCore(distance_publisher_task, "distance_publisher_task", 10000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(led_task, "led_task", 10000, NULL, 1, NULL, 1);
}

void loop()
{
    // Nothing here; all handled by FreeRTOS tasks
}
