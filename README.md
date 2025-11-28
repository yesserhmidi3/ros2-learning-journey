# ROS2 Learning Journey

In this repo I’ll talk about my ROS2 learning journey. I’ll add the source code, explain everything I did, the problems I found, and how I solved them.

For me, it’s always been hard to learn something by following a full course, watching YouTube videos, or reading long documentation. The best way I learn is simply by practicing.  
So when I decided to learn ROS2, I read a little bit of a ROS 2 Documentation documentation ([documentation link](https://docs.ros.org/en/kilted/Tutorials.html)), and I watched a video from a German YouTuber who made a simple ROS2 counter publisher just to get an idea of what I was going to do ([video link](https://www.youtube.com/watch?v=NDHHPFJnqXY)).  
After that, I looked for mini-projects. I already knew the concepts I needed to learn, so I gave ChatGPT the materials I had available and asked it to create mini projects that would help me learn ROS2 Jazzy + micro-ROS.

In this journey, I worked with an ESP32 (for micro-ROS) and Ubuntu 24.04 (ROS2 Jazzy).

---

## ROS2 Jazzy + micro-ROS Mini Projects

### 1) ESP32 Counter Publisher
**Goal:** ESP32 publishes an incrementing integer every second.  
**ROS2 part:** Create a Python node to subscribe and print the value.  
**Skills learned:** Publishing from MCU, subscribing in ROS2, agent workflow.

### 2) LED Control Subscriber
**Goal:** ESP32 subscribes to a `/led_control` topic.  
**ROS2 part:** Publish True/False to turn the onboard LED ON/OFF.  
**Skills learned:** Subscribing on MCU, controlling hardware from a ROS2 node.

### 3) Button Publisher
**Goal:** ESP32 reads a push button and publishes 1 when pressed and 0 when released.  
**ROS2 part:** Visualize button presses with `ros2 topic echo` or plot them in Python.  
**Skills learned:** Reading digital inputs, sending real-time data.

---

### 4) Potentiometer Publisher
**Goal:** ESP32 reads analog potentiometer values and publishes them.  
**ROS2 part:** Plot the values in real time.  
**Skills learned:** Analog input, data streaming, plotting.

### 5) LED Dimmer Subscriber
**Goal:** A ROS2 node publishes values between 0–255, and the ESP32 changes LED brightness using PWM.  
**Skills learned:** PWM control, mapping ROS2 messages to hardware.

### 6) Ultrasonic Sensor Publisher (Radar Plot)
(Originally a temperature sensor project, but I replaced it with the ultrasonic sensor.)  
**Goal:** ESP32 publishes distance readings.  
**ROS2 part:** Display the distance in a radar-like plot in Python.  
**Skills learned:** Sensor integration, message types, MCU → ROS2 data flow.

---


### 7) Joystick Control (Simulated)
Originally this project used a real joystick, but I simulated it using two buttons.  

Example:  
- Button1 pressed → x = 1  
- Button2 pressed → y = 1  
- Both pressed → x = 1, y = 1  
- None pressed → x = 0, y = 0  

**Skills learned:** Multi-dimensional data, ROS2 publishing/subscribing.

### 8) Bi-directional Communication
This became an ultrasonic + LED feedback system.  

Flow:  
- MCU publishes distance  
- ROS2 reads it  
- ROS2 decides which LED to turn on  
- MCU subscribes and updates LED state  

**Skills learned:** Full-duplex MCU ↔ ROS2 interaction.

---

Before going through the projects one by one, I’ll first explain how to create a ROS2 project, how to run it, and how to run a micro-ROS program.

## How to Create and Run a ROS2 Project

### 1) Source ROS2
source /opt/ros/jazzy/setup.bash

Add it to `~/.bashrc` to do it automatically:
nano ~/.bashrc
# add at the end:
source /opt/ros/jazzy/setup.bash

### 2) Create a workspace
mkdir -p ros2_ws/src
cd ros2_ws

### 3) Initialize workspace
colcon build

### 4) Source workspace
source install/setup.bash

### 5) Shortcut: build + source
Add alias to `~/.bashrc`:
nano ~/.bashrc
# add at the end:
alias ccb='colcon build && source install/setup.bash'

Now typing `ccb` builds and sources in one step.

### 6) Create a Python ROS2 package
cd src
ros2 pkg create --build-type ament_python name_of_your_package

### 7) Build workspace again
cd ..
ccb

### 8) Write ROS2 code
Open VS Code:
cd src
code .

Create a new Python file (example: `first_publisher.py`) inside your package folder.  
Do not write ROS2 code in `__init__.py`.

### 9) Add file to `setup.py`
entry_points={
    'console_scripts': [
        'first_pub = name_of_your_package.first_publisher:main',
    ],
},

Format:
"command_name = package_name.python_file_name:main"

### 10) Add dependencies in `package.xml`
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>

### 11) Build and source
ccb

### 12) Run your ROS2 program
ros2 run name_of_your_package first_pub

### 13) Check topics
ros2 topic list
ros2 topic echo /topic_name

---

## How to Run a micro-ROS Program on ESP32

1) Upload your sketch to ESP32

2) Start the micro-ROS Agent  
Source ROS2 and workspace:
source /opt/ros/jazzy/setup.bash   # or add to bashrc
source ~/microros_ws/install/local_setup.bash  # or add to bashrc

Run the agent (serial communication):
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

Keep this terminal open — it listens for the ESP32.  
Note: Since the ESP32 is connected to Arduino IDE, the agent won’t work until you close Arduino IDE and reset the ESP32.

3) In another terminal, source ROS2 and workspace again:
source /opt/ros/jazzy/setup.bash  # or bashrc
source ~/microros_ws/install/local_setup.bash  # or bashrc

To see topics from the ESP32:
ros2 topic list

To see messages being published or received:
ros2 topic echo /topic_name

4) In your ESP32 code, set up communication with the agent:
set_microros_serial_transports(Serial);

This tells the MCU: “Talk to the agent over this serial port.”

