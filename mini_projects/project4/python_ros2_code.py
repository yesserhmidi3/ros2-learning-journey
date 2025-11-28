import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
import matplotlib.pyplot as plt
from collections import deque

class potentiometer_subscriber(Node):
    def __init__(self, max_points=100):
        super().__init__('potentiometer_subscriber')
        self.subscription = self.create_subscription(
            Int32, 
            'potentiometer',  
            self.listener_callback, 
            10
        )
        self.subscription  
        self.get_logger().info("Subscriber node started")


        # Plot setup
        plt.ion()  # interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Potentiometer Readings")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Value")
        self.ax.set_ylim(0, 255)  # adjust depending on your ADC range
        self.line, = self.ax.plot([], [], 'b-o')

        # Data storage
        self.max_points = max_points
        self.data = deque(maxlen=max_points)  # store last N readings
        self.x = deque(maxlen=max_points)

        self.counter = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Add new value
        self.data.append(msg.data)
        self.x.append(self.counter)
        self.counter += 1

        # Update plot
        self.line.set_data(self.x, self.data)
        self.ax.set_xlim(max(0, self.counter - self.max_points), self.counter)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # short pause to update plot

def main():
    rclpy.init()
    node = potentiometer_subscriber()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
