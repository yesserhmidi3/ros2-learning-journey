# ESP32 micro-ROS Templates

This folder contains **three ready-to-use templates** for ESP32 using micro-ROS:

- **Publisher**
- **Subscriber**
- **Publisher + Subscriber**

These templates include all the boilerplate setup (allocator, support, node, executor, RTOS task, transport, includes, etc.).  
You **only modify a few small parts** depending on your project.

---

## What You Need to Change (Only These)

---

## 1. Message Type

In both publisher and subscriber templates:

std_msgs__msg__Int32 msg;

You can replace **Int32** with:

- Bool
- Float32
- Int16
- String (**I tried it ,not recommended**)
- Float32MultiArray
- Any supported micro-ROS message type

If the type changes, update it **everywhere the message appears**.

---

## 2. Topic Name

Inside publisher or subscriber setup:

"counter"      # change to anything you want

Make sure the ESP32 and ROS2 Python program use **the same topic name**.

---

## 3. Node Name

Inside node creation:

rclc_node_init_default(&node, "esp32_my_node", "", &support);

Rename "esp32_my_node" to anything appropriate for your project.

---

## 4. Publisher / Subscriber Setup

Update **message type + topic** inside the init calls.

### Publisher:

rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "your_topic"
);

### Subscriber:

rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "your_topic"
);

---

## 5. RTOS Task Logic

This is the part you customize depending on your behavior.

### When publishing:

Keep this line **exactly as-is**:

rcl_publish(&publisher, &msg, NULL);

Modify only how you compute or update:

msg.data;

### When subscribing:

Keep the cast identical:

const std_msgs__msg__Int32 *in_msg = (const std_msgs__msg__Int32 *) msgin;

Use:

in_msg->data

to read the value.

---

## Everything Else = Don’t Touch

Keep the following unchanged:

- rcl_allocator_t, rclc_support_init()
- rclc_executor_init()
- xTaskCreatePinnedToCore(...)
- set_microros_transports()
- Includes and global variables
- loop() function (always empty)

These are boilerplate required for micro-ROS. You simply reuse them.

---

## Summary

When creating a new micro-ROS ESP32 project, you only edit **5 things**:

1. Message type  
2. Topic name  
3. Node name  
4. Publisher/subscriber init line  
5. Logic inside your RTOS task (publish/subscribe behavior)

Everything else stays unchanged so you can move fast and avoid repeating boilerplate.

