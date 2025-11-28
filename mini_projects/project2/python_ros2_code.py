# ===== Step 0: Import ROS2 libraries =====
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ===== Step 1 & 2: Define your node class =====
class LEDPublisher(Node):
    def __init__(self):
        super().__init__('led_control_publisher')  # Node name

        # Publisher for the /led_control topic
        self.publisher_ = self.create_publisher(Bool, 'led_control', 10)
        self.get_logger().info("LED Publisher node started")

        # ===== OLD BLINKING TIMER (commented out) =====
        # self.led_state = False
        # self.timer_ = self.create_timer(1.0, self.timer_callback)  # 1 sec period

    # ===== Dynamic user input method =====
    def user_input(self):
        while True:
            user_cmd = input("Type 0/1 to turn LED off/on: ")
            msg = Bool()
            if user_cmd == '1':
                msg.data = True
            elif user_cmd == '0':
                msg.data = False
            else:
                print("Invalid input. Please type 0 or 1.")
                continue
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing LED state: {msg.data}')

    # ===== OLD BLINKING TIMER CALLBACK (commented out) =====
    # def timer_callback(self):
    #     msg = Bool()
    #     msg.data = self.led_state
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'Publishing LED state: {msg.data}')
    #     self.led_state = not self.led_state

# ===== Step 3: Main function =====
def main():
    rclpy.init()
    node = LEDPublisher()
    try:
        # Run dynamic user input
        node.user_input()

        # ===== OLD TIMER-SPIN (commented out) =====
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown
    node.destroy_node()
    rclpy.shutdown()

# ===== Step 4: Run main =====
if __name__ == '__main__':
    main()
