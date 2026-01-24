# RViz Templates

This is a **template ROS2 Python node** for publishing markers to **RViz2**. It provides a starting point for visualizing points, shapes, or simple animations in RViz using the `visualization_msgs/Marker` message type.  

---

## What It Does

- Publishes a **SPHERE marker** on the `/visualization_marker` topic.  
- Moves the marker along the X-axis in a simple animation loop.  
- Updates markers in real time with a timer callback (0.5-second interval).  
- Demonstrates how to set marker properties such as:
  - Position (`pose.position`)  
  - Orientation (`pose.orientation`)  
  - Scale (`scale`)  
  - Color (`color`)  
  - Lifetime (`lifetime`)  
  - Marker type and action  

---

## How to Run

1. Make sure your ROS2 workspace is sourced:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Run the node:
```bash
ros2 run <your_package_name> rviz_template
```

3. Open **RViz2** to visualize the marker:
```bash
rviz2
```
- Set the **Fixed Frame** to `map`.
- Add a **Marker** display and subscribe to the topic `/visualization_marker`.
You should see a red sphere moving along the X-axis.

---

## Concepts Reinforced
- Publishing `Marker` messages in ROS2  
- Setting marker properties: type, color, scale, pose, and lifetime  
- Animating markers with a timer callback  
- Visualizing ROS2 topics in RViz2  

---

## How to Customize
- Change `marker.type` to visualize other shapes (CUBE, ARROW, LINE_STRIP, etc.)  
- Modify `marker.color` to change the color or transparency  
- Adjust `marker.scale` for size  
- Change the movement logic in `publish_marker()` to animate differently  
- Add multiple markers with unique `marker.id` and `marker.ns`  
