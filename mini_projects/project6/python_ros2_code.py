import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Change to the message type you expect
import matplotlib.pyplot as plt # Optional: for radar like visualization (0..360°)
import numpy as np # Optional: for numerical operations

class distance_subscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(
            Float32, 
            'distance', 
            self.listener_callback, 
            10
        )
        self.subscription  # Avoid unused variable warning
        self.get_logger().info("Subscriber node started")
    
        plt.ion()  # Interactive mode on for live updating plots

        #set up radar plot
        self.fig = plt.figure() # Create a new figure
        self.ax = self.fig.add_subplot(111, polar=True) # Add a polar subplot

        #set max radius
        self.ax.set_rmax(200) # Adjust based on expected max distance
        self.ax.set_rticks([50, 100, 150, 200])  # Radial ticks
        self.ax.set_theta_zero_location('N')  # 0° at the top
        self.ax.set_theta_direction(-1)  # Clockwise
        plt.title("Distance Sensor Readings", va='bottom') # Add title
        plt.show() # Show the plot window
    
        self.angle = 0  # Initialize angle for radar plot
    
    def listener_callback(self, msg):
        distance = msg.data 
        theta = np.deg2rad(self.angle)  # Convert angle to radians

        self.ax.plot(theta,distance,'ro') # Plot the point
        self.ax.plot([0, theta], [0, distance], 'g-') # Line from center to point
        plt.pause(0.05) # Pause to update the plot
        
        self.angle += 5 # Increment angle for next reading

        if self.angle >= 360:
            self.angle = 0


        self.get_logger().info(f'Received: {msg.data}')
        
        # You can add logic here to process incoming data

def main():
    rclpy.init()
    node = distance_subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
