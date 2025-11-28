# Project 3 — Button Publisher

ESP32 reads a push button and publishes 1 when pressed, 0 when released.

- Topic: `/button`  
- Message type: `std_msgs/msg/Int32`  
- ROS2 Python node subscribes and prints the button state

**Files:**  
- `esp32_button_pub/` → ESP32 publisher C code  
- `ros2_button_sub/` → Python subscriber  

**Goal:** Learn reading digital inputs and publishing to ROS2.
