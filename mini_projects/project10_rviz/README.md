# Multi-Sensor Visualization Project (RViz)

This project demonstrates **real-time visualization of multiple sensor data** from an ESP32 using ROS2 and RViz.

We use:

- **Potentiometer** → Red cube height proportional to analog value
- **Push Button** → Green cube appears when pressed
- **Ultrasonic Distance Sensor** → Blue cube height proportional to measured distance

---

## How It Works

- The **ESP32** publishes sensor data to three ROS2 topics:
  - `/pot_value` (Int32)
  - `/button_state` (Bool)
  - `/distance_cm` (Float32)
- The **ROS2 node** subscribes to these topics and publishes corresponding **RViz markers**:
  - Red cube → Potentiometer
  - Green cube → Button
  - Blue cube → Distance sensor
- Markers are published on topics:
  - `pot_marker`, `button_marker`, `distance_marker`

> The ROS2 marker setup uses the same **RViz template** explained in the `templates/rviz` folder.  

---

## How to Run

1. **Start the micro-ROS agent** (UDP example):
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
2. **Upload ESP32 code** to the board. Make sure WiFi credentials and micro-ROS IP/port match your network.
3. **Run the ROS2 node** on your PC:
   ```bash
   ros2 run multi_sensor_project multi_sensor_sub
   ```
4. **Open RViz** and add a `Marker` display for each topic:
   - `pot_marker`
   - `button_marker`
   - `distance_marker`

---

## Notes / Attention

- Cube heights are **normalized** (potentiometer: `/255`, distance: `/100`) to fit RViz scaling.
- Button marker **deletes the cube** when released for better visualization.
- Topics and frame names (`base_link`) must match RViz configuration.

> For more details on RViz marker setup, see the [RViz template README](../templates/rviz/README.md)
