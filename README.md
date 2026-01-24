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

### 9) OpenCV + MediaPipe ROS2 Project (Personal project)

After finishing the mini-projects, I created a small personal project to see what I could do with ROS2 and apply some computer vision.  

**Goal:** Use a camera to detect hand landmarks with MediaPipe and send commands to control LEDs on the ESP32 via micro-ROS.  

**Skills learned:**  
- Integrating OpenCV and MediaPipe with ROS2  
- Debugging library installation and compatibility issues  
- Applying ROS2 concepts to a real-world scenario   

---
---

## Beyond micro-ROS: Visualization & Simulation

After finishing the micro-ROS mini-projects, I wanted to go further than just
seeing values in the terminal.

At this point, I already understood:
- ROS2 nodes, topics, publishers, subscribers
- MCU ↔ ROS2 communication
- Sensors and actuators

So the next natural steps were:
1. Visualizing data properly using RViz
2. Simulating robots and control systems using Gazebo

### 10) Multi-Sensor Visualization with RViz
In this project, I combined multiple sensors connected to the ESP32 and visualized
their data in RViz.

**Sensors used:**
- Potentiometer
- Push button
- Ultrasonic distance sensor
  
**What I did:**
- Created a ROS2 package called `multi_sensor`
- Published sensor data from the ESP32 using micro-ROS
- Subscribed to the sensor topics in ROS2
- Visualized:
  - Button state
  - Analog values
  - Distance readings
  using RViz

**Skills learned:**
- Using RViz for real-time data visualization
- Mapping sensor data to RViz displays
- Debugging sensor topics visually instead of using the terminal
- Understanding how ROS2 tools fit into a real workflow


### 11) Self-Balancing Robot Simulation (Gazebo + ROS2)

After working with real hardware and visualization tools, I wanted to explore
robot simulation and control using Gazebo.

The goal of this project was to simulate a self-balancing robot and control it
using a PID controller written in ROS2.

#### Robot Modeling (URDF)
I created a URDF model of a self-balancing robot and:
- Added inertial properties to each link to define physical behavior
- Defined joints and their limits
- Added motors to the joints for actuation

#### Gazebo Integration
- Used Gazebo Harmonic (ROS2 Jazzy compatible)
- Added Gazebo-specific sensors (IMU)
- Configured motor interfaces
- Created a launch file to:
  - Spawn the robot into the world
  - Load the URDF
  - Bridge Gazebo topics to ROS2

#### ROS2 Control & PID
I created a `main.py` ROS2 node that:
- Subscribes to the IMU data published by Gazebo
- Computes the robot’s tilt angle
- Applies a classic PID controller
- Publishes motor commands back to Gazebo

This closed the control loop:
Gazebo → ROS2 → PID → Gazebo

**Skills learned:**
- Gazebo ↔ ROS2 topic bridging
- Writing control logic in ROS2
- Implementing and tuning a PID controller
- Understanding robot dynamics in simulation




Before going through the projects one by one, I’ll first explain how to create a ROS2 project, how to run it, and how to run a micro-ROS program.  

## How to Create and Run a ROS2 Project  

### 1) Source ROS2  
source /opt/ros/jazzy/setup.bash  

Add it to `~/.bashrc` to do it automatically:  
nano ~/.bashrc  
add at the end:  
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
add at the end:  
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

### 13) Check topics (optional)  
ros2 topic list  
ros2 topic echo /topic_name  

---

## How to Run a micro-ROS Program on ESP32  

### 1) Upload your sketch to ESP32  

### 2) Start the micro-ROS Agent  
Source ROS2 and workspace:  
source /opt/ros/jazzy/setup.bash   # or add to bashrc  
source ~/microros_ws/install/local_setup.bash  # or add to bashrc  

Run the agent (serial communication):  
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0  

Keep this terminal open — it listens for the ESP32.  
Note: Since the ESP32 is connected to Arduino IDE, the agent won’t work until you close Arduino IDE and reset the ESP32.  

### 3) In another terminal, source ROS2 and workspace again:  
source /opt/ros/jazzy/setup.bash  # or bashrc  

source ~/microros_ws/install/local_setup.bash  # or bashrc  

To see topics from the ESP32:  
ros2 topic list  

To see messages being published or received:  
ros2 topic echo /topic_name  

### 4) In your ESP32 code, set up communication with the agent:  
set_microros_serial_transports(Serial);  

This tells the MCU: “Talk to the agent over this serial port.”  

---

## How to Create and Run a ROS2 Project 

### 1) Create a workspace 
mkdir -p gazebo_ws/src
cd gazebo_ws

### 2) Build and source the workspace 
colcon build
source install/setup.bash

(Or use the ccb alias if you added it earlier.)

### 3) Create a ROS2 Python package
cd src
ros2 pkg create --build-type ament_python name_of_your_package

### 4) Create the project folders 
Inside your package directory, create the following folders:
cd name_of_your_package
mkdir urdf launch

Optional :
mkdir world

. urdf/ → robot description files (.urdf or .xacro)
. launch/ → launch files
. world/ → Gazebo world files (.sdf)

### 5) Add your URDF and launch files
Create your robot URDF inside urdf/

Create your launch file inside launch/

Follow the provided Gazebo templates for structure and best practices

Make sure your URDF includes:

Inertial properties

Joints and motors

Gazebo sensor plugins (IMU, etc.) 

### 6) Register folders in setup.py (IMPORTANT)  
To make sure ROS2 can find your URDF and launch files, you must register them in setup.py.

At the top of setup.py, add:
import os
from glob import glob

Then update data_files like this:
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    # Register launch and URDF folders
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
],

If you have other folders (e.g. world/), register them the same way.

### 7) Add dependencies in package.xml  
Add the required dependencies:

<depend>rclpy</depend>
<depend>xacro</depend>
<depend>gazebo_ros</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>

<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>

Only add what you actually use in your project.

### 8) Write your control node (main.py)
.Create your main.py file inside the package

.Follow the Gazebo main control template

.Typical responsibilities:
    .Subscribe to sensor topics (e.g. IMU)
    .Compute control logic (PID, etc.)
    .Publish commands to motors

Don’t forget to register the node in setup.py under console_scripts.

### 9) Build and source
cd ~/gazebo_ws
ccb

### 10) Launch the simulation
In one terminal:

ros2 launch name_of_your_package launch_file_name.py

This will:
    .Start Gazebo
    .Spawn the robot
    .Bridge Gazebo topics to ROS2

### 11) Run the control node  
In another terminal:

ros2 run name_of_your_package main_file_name

This starts your ROS2 control loop (PID, logic, etc.).


## Code Templates

After completing the 3rd mini-project, I created, with the help of ChatGPT, **code templates** to copy and paste at the beginning of each project.  
This way, we don’t waste time writing everything from scratch, and we don’t have to memorize all the functions we will use.  

I’ll explain each template, what it does, and how to modify it based on your needs. It’s impossible to write all of this from memory, so having templates is extremely useful.  

There are **6 templates** in total:

### ESP32 micro-ROS C Code
1. **Publisher** – basic template for sending messages from the MCU to ROS2  
2. **Subscriber** – basic template for receiving messages from ROS2  
3. **Publisher + Subscriber** – combination template for full-duplex communication

### Python ROS2 Code
1. **Publisher** – basic ROS2 Python publisher  
2. **Subscriber** – basic ROS2 Python subscriber  
3. **Publisher + Subscriber** – combination template for full-duplex communication

I started using these templates beginning from project 4, and they made the workflow much faster and cleaner.  
Each template includes all the necessary setup and boilerplate code. You can copy it directly, understand each part, and then adapt it to your project’s goals.  

### RViz Template

This template provides a ready-to-use RViz configuration for visualizing ROS2 topics.

**What it includes:**
- Preconfigured RViz displays
- Topic mappings for common message types
- Fixed frame setup
- A clean visualization layout

**Use case:**
- Quickly visualize sensor data (buttons, analog values, distance, IMU, etc.)
- Debug ROS2 topics visually instead of using the terminal

**Skills reinforced:**
- Understanding frames and topics
- Using RViz as a debugging and development tool

### Gazebo Templates
#### 1) URDF Template
A reusable URDF template for creating simulated robots.

**What it includes:**
- Link and joint structure
- Inertial properties
- Motor definitions
- Gazebo sensor plugins (e.g., IMU)

**Use case:**
- Quickly define physically realistic robots
- Avoid rewriting boilerplate URDF and Gazebo tags

**Skills reinforced:**
- Robot modeling
- Understanding robot dynamics
- Simulation realism
#### 2) Launch File Template
A ROS2 launch file template to:
- Spawn a robot into a Gazebo world
- Load the URDF
- Bridge Gazebo topics to ROS2
#### 3) Main Control Node Template
A ROS2 Python node template for controlling simulated robots.

**What it includes:**
- Subscription to sensor topics (e.g., IMU)
- Publishing commands to actuators
- PID control structure (easily tunable)

**Use case:**
- Rapidly implement control logic
- Focus on control algorithms instead of ROS2 boilerplate

**Skills reinforced:**
- Control loops
- PID control
- Closed-loop simulation


---

## Repository Overview / Folder Structure  

Here’s how the repository is organized:  

/templates  
    /python_ros2       # Publisher, Subscriber, Publisher+Subscriber templates  
    /esp32_microros    # Publisher, Subscriber, Publisher+Subscriber templates  
    /rviz
    /gazebo
        /urdf
        /launch
        /control

/mini_projects  
    /project1          # ESP32 Counter Publisher  
    /project2          # LED Control Subscriber  
    /project3          # Button Publisher  
    /project4          # Potentiometer Publisher  
    /project5          # LED Dimmer Subscriber  
    /project6          # Ultrasonic Sensor Publisher  
    /project7          # Joystick Control  
    /project8          # Bi-directional Communication  
    /project9          # OpenCV + MediaPipe ROS2 Project  
    /project10_rviz        # Multi-sensor visualization with RViz
    /project11_gazebo      # Self-balancing robot simulation
        /urdf
        /launch
        /scripts

README.md

This structure helps you quickly find the templates and mini-projects, and see how everything fits together.  

---

## Future Work / Updates

This repo is still a work in progress. I’ll keep updating it with:   
- New mini-projects to explore more ROS2 concepts 
- Better or new templates for Python ROS2 and ESP32 micro-ROS  
- Tips and solutions for any issues I run into while learning  

This is my ROS2 learning journey, so it’s constantly evolving.  
The goal is to make it a useful reference for anyone who wants to learn ROS2 and micro-ROS.  



