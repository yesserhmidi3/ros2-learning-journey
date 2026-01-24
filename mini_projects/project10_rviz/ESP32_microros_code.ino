#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include <WiFi.h>
// ---------- micro-ROS Globals ----------
rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t node;
rcl_publisher_t publisher_potentiometer;
rcl_publisher_t publisher_button;
rcl_publisher_t publisher_distance;

rclc_executor_t executor;

std_msgs__msg__Int32 msgPot;
std_msgs__msg__Bool msgButton;
std_msgs__msg__Float32 msgUltras;

// ---------- Pins ----------
const int trigPin = 5;
const int echoPin = 18;
const int BUTTON_PIN = 22;
const int potPin = 15;

#define SOUND_SPEED 0.034
long duration;
float distance;

// ---------- Tasks ----------
TaskHandle_t potentiometer_task_handle;
TaskHandle_t button_task_handle;
TaskHandle_t distance_task_handle;

// ---------- Potentiometer Task ----------
void potentiometer_task(void *pvParameters)
{
    while (1)
    {
        msgPot.data = (analogRead(potPin) * 255) / 4095;
        rcl_publish(&publisher_potentiometer, &msgPot, NULL);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ---------- Button Task ----------
void button_task(void *pvParameters)
{
    while (1)
    {
        msgButton.data = (digitalRead(BUTTON_PIN) == LOW);
        rcl_publish(&publisher_button, &msgButton, NULL);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ---------- Distance (Ultrasound) Task ----------
void distance_task(void *pvParameters)
{
    while (1)
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);

        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);

        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        distance = duration * SOUND_SPEED / 2.0f;

        msgUltras.data = distance;
        rcl_publish(&publisher_distance, &msgUltras, NULL);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    WiFi.begin("Orange-7894", "14J8FFLFG8A");
    while (WiFi.status() != WL_CONNECTED) delay(100);
    //set_microros_transports();
    set_microros_wifi_transports("Orange-7894", "14J8FFLFG8A", "192.168.1.187", 8888); //ip address  = hostname -I
    //Terminal : ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

    pinMode(potPin, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    delay(2000);

    // ----- micro-ROS Init -----
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // ----- ONE Node -----
    rclc_node_init_default(&node, "multi_sensor_node", "", &support);

    // ----- Publishers -----
    rclc_publisher_init_default(
        &publisher_potentiometer,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pot_value");

    rclc_publisher_init_default(
        &publisher_button,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "button_state");

    rclc_publisher_init_default(
        &publisher_distance,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "distance_cm");

    // ----- Executor -----
    rclc_executor_init(&executor, &support.context, 3, &allocator);

    // ----- Create FreeRTOS Tasks -----
    xTaskCreatePinnedToCore(potentiometer_task, "pot_task", 4096, NULL, 2, &potentiometer_task_handle, 1);
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 1, &button_task_handle, 1);
    xTaskCreatePinnedToCore(distance_task, "distance_task", 4096, NULL, 3, &distance_task_handle, 1);
}

void loop()
{
    // Nothing here, tasks run independently
}
