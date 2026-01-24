import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Imu
import math

class Esp32GazeboBridge(Node):
    def __init__(self):
        super().__init__('esp32_gazebo_bridge')
        # PID Constants (You will need to tune these!)
        self.Kp = 40
        self.Ki = 0
        self.Kd = 5
        self.last_pitch = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # 1. Subscriber to Gazebo IMU (bridged in launch file)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # 2. Publisher to Gazebo Motors
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Bridge Started: ESP32 Counter -> Gazebo Velocity")

    def imu_callback(self, msg):
    # 1. Get current time for delta_t
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # 2. Get Pitch (from Accelerometer/Orientation)
        q = msg.orientation
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2) # This is the standard Pitch formula

        if abs(math.degrees(pitch)) > 45.0:
            self.pub.publish(Twist()) # Send 0 velocity
            return

        # 3. Get Pitch Velocity (from Gyroscope)
        # Assuming Y-axis is the axis of falling for your URDF
        pitch_velocity = msg.angular_velocity.y

        # 4. PID Calculation
        error = pitch
        self.integral += error * dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * pitch_velocity)

        # 5. Deadzone compensation (The "Kick")
        if output > 0:
            output += 0.5  # Add a minimum voltage to overcome friction
        elif output < 0:
            output -= 0.5

        # Limit the speed to something realistic
        if output > 5.0: output = 5.0
        if output < -5.0: output = -5.0
        # 4. Apply to Robot
        cmd = Twist()
        cmd.linear.x = output# Direction depends on your URDF orientation
        self.get_logger().info(f"Pitch: {math.degrees(pitch):.1f} | Command: {cmd.linear.x:.2f}") # Log for debugging
        self.pub.publish(cmd)

        # Save state
        self.last_pitch = pitch
        self.last_time = curr_time

       

def main(args=None):
    rclpy.init(args=args)
    node = Esp32GazeboBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()