# Project 1 — ESP32 Counter Publisher

This project uses an **ESP32 with micro-ROS** to publish an incrementing counter to the ROS2 topic `/counter`.

- ESP32 publishes one integer every second 
- Message type: `std_msgs/msg/Int32` 
- ROS2 Python node subscribes and prints the value

### Files
- `esp32_counter_pub/` → ESP32 C code 
- `ros2_counter_sub/` → ROS2 Python subscriber 

### How to run
1. Flash the ESP32 with the publisher code 
2. Run the Python subscriber node 
3. You should see: 
Received: 0
Received: 1
Received: 2 ...... 


### Goal
Learn the **basic micro-ROS → ROS2 topic communication** (publisher → subscriber).


