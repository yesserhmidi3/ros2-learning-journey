# Project 5 — LED Dimmer Subscriber

ESP32 controls LED brightness using PWM based on values from ROS2.

- Topic: `/led_dimmer`  
- Message type: `std_msgs/msg/Int32` (0–255)  
- Python ROS2 node publishes brightness values

**Files:**  
- `esp32_led_dimmer_sub/` → ESP32 subscriber C code  
- `ros2_led_dimmer_pub/` → Python publisher  

**Goal:** Learn PWM control and mapping ROS2 messages to hardware outputs.
