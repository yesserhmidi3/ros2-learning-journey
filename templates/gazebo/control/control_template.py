import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu # Example: change to your sensor type

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        
        # 1. INPUT: Subscribe to Sensors
        self.sub = self.create_subscription(Imu, '/imu', self.control_loop, 10)
        
        # 2. OUTPUT: Publish to Actuators
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. LOGIC: Control Variables (PID)
        self.target = 0.0  # Desired state
        self.kp = 1.0      # Proportional Gain
        self.ki = 0.0      # Integral Gain
        self.kd = 0.1      # Derivative Gain
        
        self.prev_error = 0.0
        self.integral = 0.0

    def control_loop(self, msg):
        # --- A. Read Sensor ---
        # Example: Using orientation for a self-balancing robot
        current_value = msg.orientation.x 
        
        # --- B. Compute PID Logic ---
        error = self.target - current_value
        self.integral += error
        derivative = error - self.prev_error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        
        # --- C. Send Action ---
        cmd = Twist()
        cmd.linear.x = float(output) 
        self.pub.publish(cmd)
        
        # Log data for debugging
        self.get_logger().info(f'Error: {error:.2f} | Output: {output:.2f}')

def main():
    rclpy.init()
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
