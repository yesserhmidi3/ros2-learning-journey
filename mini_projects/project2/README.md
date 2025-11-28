# Project 2 — LED Control Subscriber

ESP32 subscribes to `/led_control` and turns the onboard LED ON/OFF.

- Message type: `std_msgs/msg/Bool`  
- ROS2 Python node publishes True/False to control the LED

**Files:**  
- `esp32_led_sub/` → ESP32 subscriber C code  
- `ros2_led_pub/` → Python publisher  

**Goal:** Learn subscribing on MCU and controlling hardware from ROS2.
