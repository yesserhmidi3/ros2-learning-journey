# ===== Step 0: Import ROS2 libraries =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool # Change this to other message types if needed

# ===== Step 1 & 2: Define your node class =====
class button_subscriber(Node):
    def __init__(self):
        super().__init__('button_subscriber')  # Change the node name here

        self.subscription = self.create_subscription(
            Bool, 
            'button', 
            self.listener_callback, 
            10
        )
        self.subscription  # Prevent unused variable warning

    # ===== Subscriber callback =====
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')  # Handle incoming message
        if msg.data:
            print("Button pressed!")
        else:
            print("Button released.")


# ===== Step 3: Main function =====
def main():
    rclpy.init()  # Initialize ROS2
    node = button_subscriber()
    rclpy.spin(node)  # Keep the node alive
    node.destroy_node()  # Cleanup
    rclpy.shutdown()  # Shutdown ROS2

# ===== Step 4: Run main =====
if __name__ == '__main__':
    main()
