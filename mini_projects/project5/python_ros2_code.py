import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  

class LED_Dimmer_publisher(Node):
    def __init__(self):
        super().__init__('LED_Dimmer_publisher') # Initialize the node with the name 'LED_Dimmer_publisher'
        self.publisher_ = self.create_publisher(Int32, 'led', 10) # Create a publisher on the 'led' topic with message type Int32
        self.get_logger().info("Publisher node started") # Log that the publisher node has started




def main():
    rclpy.init() # Initialize the ROS2 Python client library
    node = LED_Dimmer_publisher() # Create an instance of the LED_Dimmer_publisher node
    try: 
        while rclpy.ok(): # Keep running while ROS2 is operational
            value = input("Enter LED brightness (0-255) ")  
            try:
                value_int = int(value)
            except: 
                node.get_logger().error("Invalid input. Please enter an integer between 0 and 255.") 
                continue
            msg = Int32()
            msg.data = value_int
            node.publisher_.publish(msg) # Publish the message
            node.get_logger().info(f"Published LED brightness: {value_int}") # Log the published brightness value


    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
