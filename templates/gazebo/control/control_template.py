import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu # Change this based on your sensor

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # 1. Pub/Sub Setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Imu, '/sensor_data', self.callback, 10)
        
        # 2. Control Variables
        self.target = 0.0
        self.kp = 1.0 # Tune me!

    def callback(self, msg):
        # 3. Read Sensor (Processing Logic)
        current_value = msg.orientation.x # Example
        
        # 4. Control Logic (The Brain)
        error = self.target - current_value
        output = self.kp * error
        
        # 5. Send Action
        cmd = Twist()
        cmd.linear.x = float(output)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()
    