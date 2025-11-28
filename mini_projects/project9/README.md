# Project 9 — OpenCV + MediaPipe ROS2 Project

This project uses a Python ROS2 node to detect hand landmarks with MediaPipe and control LEDs on an ESP32 via micro-ROS.

- Topics: `/hand_landmarks` (publisher), `/led_control` (subscriber)  
- ESP32 receives commands to light LEDs based on hand gestures

---

## ⚠️ Notes / Problems & Solutions

When I first tried this project, I ran into a problem installing MediaPipe:

1. **Direct `pip install` warning**  
   - If you run `pip install mediapipe` on Ubuntu without a venv, you may get a warning.  
   - Forcing it works, but **it can break the system Python3**, so it’s not recommended.

2. **Using a Virtual Environment**  
   - The recommended approach is to create a Python venv:
     ```bash
     python3 -m venv ~/venvs/ros2_cv
     source ~/venvs/ros2_cv/bin/activate
     pip install opencv-python mediapipe
     ```
   - At first, ROS2 could not see the packages even when running inside the venv.

3. **Proper ROS2 build inside venv**  
   - The default `colcon build` uses the system Python, so it won’t see your venv packages.  
   - The solution is to **use the venv Python to build**:
     ```bash
     python -m colcon build
     ```

---

## 🐍 How to Run

1. Activate your venv:
   ```bash
   source ~/venvs/ros2_cv/bin/activate
2. Run the Python ROS2 node:
   ```bash
    ros2 run ros2_opencv_mediapipe hand_tracker
3. Run the micro-ROS agent and ESP32 subscriber as usual.

**Goal:** Combine computer vision, ROS2 messaging, and micro-ROS control.  
Combine computer vision, ROS2 messaging, and micro-ROS control, while learning proper Python environment management for ROS2 projects and handling library installation issues safely.


