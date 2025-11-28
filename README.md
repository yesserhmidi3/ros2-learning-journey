# ros2-learning-journey
My personal journey learning ROS2 and micro-ROS through small practical mini-projects, notes, and examples.


# ROS2 Learning Journey

In this repo I’ll talk about my ROS2 learning journey. I’ll add the source code, explain everything I did, the problems I found, and how I solved them.

For me, it’s always been hard to learn something by following a full course, watching YouTube videos, or reading long documentation. The best way I learn is simply by practicing.  
So when I decided to learn ROS2, I read a little bit of the official ROS2 Jazzy documentation (documentation link), and I watched a video from a German YouTuber who made a simple ROS2 counter publisher just to get an idea of what I was going to do (video link).  
After that, I looked for mini-projects. I already knew the concepts I needed to learn, so I gave ChatGPT the materials I had available and asked it to create mini projects that would help me learn ROS2 Jazzy + micro-ROS.

In this journey, I worked with an ESP32 (for micro-ROS) and Ubuntu 24.04 (ROS2 Jazzy).

---

## ROS2 Jazzy + micro-ROS Mini Projects

### Beginner Level — Getting Comfortable with Topics

#### 1) ESP32 Counter Publisher
**Goal:** ESP32 publishes an incrementing integer every second.  
**ROS2 part:** Create a Python node to subscribe and print the value.  
**Skills learned:** Publishing from MCU, subscribing in ROS2, agent workflow.

#### 2) LED Control Subscriber
**Goal:** ESP32 subscribes to a `/led_control` topic.  
**ROS2 part:** Publish True/False to turn the onboard LED ON/OFF.  
**Skills learned:** Subscribing on MCU, controlling hardware from a ROS2 node.

#### 3) Button Publisher
**Goal:** ESP32 reads a push button and publishes 1 when pressed and 0 when released.  
**ROS2 part:** Visualize button presses with `ros2 topic echo` or plot them in Python.  
**Skills learned:** Reading digital inputs, sending real-time data.

---

### Intermediate Level — Sensors and Actuators

#### 4) Potentiometer Publisher
**Goal:** ESP32 reads analog potentiometer values and publishes them.  
**ROS2 part:** Plot the values in real time.  
**Skills learned:** Analog input, data streaming, plotting.

#### 5) LED Dimmer Subscriber
**Goal:** A ROS2 node publishes values between 0–255, and the ESP32 changes LED brightness using PWM.  
**Skills learned:** PWM control, mapping ROS2 messages to hardware.

#### 6) Ultrasonic Sensor Publisher (Radar Plot)
(Originally a temperature sensor project, but I replaced it with the ultrasonic sensor.)  
**Goal:** ESP32 publishes distance readings.  
**ROS2 part:** Display the distance in a radar-like plot in Python.  
**Skills learned:** Sensor integration, message types, MCU → ROS2 data flow.

---

### Advanced Level — Combining MCU + ROS2

#### 7) Motor Speed Controller
**Goal:** ESP32 subscribes to a `/motor_speed` topic and controls a DC motor via PWM.  
**ROS2 part:** A Python node publishes speed values.  
**Skills learned:** Real-time control, ROS2 → MCU communication.

#### 8) Joystick Control (Simulated)
Originally this project used a real joystick, but I simulated it using two buttons.  

Example:  
- Button1 pressed → x = 1  
- Button2 pressed → y = 1  
- Both pressed → x = 1, y = 1  
- None pressed → x = 0, y = 0  

**Skills learned:** Multi-dimensional data, ROS2 publishing/subscribing.

#### 9) Bi-directional Communication
This became an ultrasonic + LED feedback system.  

Flow:  
- MCU publishes distance  
- ROS2 reads it  
- ROS2 decides which LED to turn on  
- MCU subscribes and updates LED state  

**Skills learned:** Full-duplex MCU ↔ ROS2 interaction.

---

Before going through the projects one by one, I’ll first explain how to create a ROS2 project, how to run it, and how to run a micro-ROS program.
