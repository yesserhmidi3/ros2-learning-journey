# ROS2 Python Templates (Jazzy)

This folder contains **three ready-to-use templates** for ROS2 Python:

- **Publisher**
- **Subscriber**
- **Publisher + Subscriber**

These templates include all the boilerplate setup (rclpy init, node creation, publishers/subscribers, spin loops, imports, etc.).  
You **only modify a few small parts** depending on your project.

---

## What You Need to Change (Only These)

---

## 1. Message Type

In both publisher and subscriber templates:

```python
from std_msgs.msg import Int32
msg = Int32()
You can replace Int32 with any standard ROS2 message type:

Bool

Float32

Int16

String

Float32MultiArray

Any supported ROS2 message

If the type changes, update it everywhere the message appears.

---

## 2. Topic Name
Inside publisher or subscriber setup:

python
Copy code
self.publisher_ = self.create_publisher(Int32, 'counter', 10)
Change 'counter' to anything you want.
Make sure the ROS2 Python nodes use the same topic name.

---

## 3. Node Name
Inside node creation:

python
Copy code
rclpy.init()
node = rclpy.create_node('my_node')
Rename 'my_node' to anything appropriate for your project.

---

## 4. Publisher / Subscriber Setup
Update message type + topic inside the init calls.

Publisher:
python
Copy code
self.publisher_ = self.create_publisher(Int32, 'your_topic', 10)
Subscriber:
python
Copy code
self.subscription = self.create_subscription(
    Int32,
    'your_topic',
    self.listener_callback,
    10
)

---

## 5. Callback / Publish Logic
When publishing:
python
Copy code
msg.data = value  # compute or update value
self.publisher_.publish(msg)
When subscribing:
python
Copy code
def listener_callback(self, msg):
    print("Received:", msg.data)
Use msg.data to access the value.

---

## Everything Else = Don’t Touch
Keep the following unchanged:

rclpy.init() and rclpy.spin(node)

Node class boilerplate

Import statements for ROS2 packages

Shutdown code: rclpy.shutdown()

These are standard ROS2 Python boilerplate. You simply reuse them.

---

## Summary
When creating a new ROS2 Python project, you only edit 5 things:

Message type

Topic name

Node name

Publisher/subscriber init line

Logic inside your callbacks or publishing loop

Everything else stays unchanged so you can move fast and avoid repeating boilerplate.
