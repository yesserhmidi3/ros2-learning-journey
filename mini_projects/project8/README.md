# Project 8 — Bi-directional Communication

ESP32 publishes ultrasonic sensor data and subscribes to control LEDs.

- Topics: `/ultrasonic` (publisher), `/led_control` (subscriber)  
- Python ROS2 node processes data and sends commands back to ESP32

**Files:**  
- `esp32_bidir/` → ESP32 pub + sub C code  
- `ros2_bidir/` → Python pub + sub  

**Goal:** Learn full-duplex communication between MCU and ROS2.
