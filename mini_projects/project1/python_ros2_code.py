#Step 0 : Import ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#Step 1 : Initialize ROS2 =>rclpy.init() **Start the ROS2 client**
rclpy.init()
#Think: “Boot up the ROS2 engine before creating nodes.”

#Step 2 : Create a node =>rclpy.create_node('name') **Give your program a ROS identity**
#In ROS2 Python, we usually define a Node as a class — this makes it easier to expand later
#node = rclpy.create_node('Name') => class MyNode(Node): super().__init__('name')
class counter_subscriber(Node):#create a node and gives it a name
    def __init__(self): #self =current node
        super().__init__('esp32_counter_receiver')
    #Parameters:
    #'esp32_counter_receiver': The name of your node as it will appear in ROS2 (run ros2 node list to see it).
    #Think: “This is my identity in the ROS2 network.”
        ##Step 3 : Create a subscriber =>create_subscription(msg_type, topic_name, callback, qos) **Listen to /counter topic**
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning / prevents : “Function result is not used” error
        #Parameters:
        #Int32 : Type of message expected → Int32 (from std_msgs.msg)
        #'counter' : Name of the topic to listen to → 'counter'
        #callback : Function that runs every time a new message arrives
        #qos : Queue size (how many messages can be buffered if they come fast) → 10
        #Think: “Whenever someone publishes on /counter, run my callback and give me the message.”
    
    #Step 4 : Define a callback =>def callback(msg): ... **Runs automatically every time a new message is received.**
    def callback(self, msg):
        self.get_logger().info(f'Received counter value: {msg.data}') #prints a message to the ROS2 log system
    #Parameters:
    #msg : The message object received. It has one field here: msg.data, which holds the integer sent by ESP32.
    #Think: “Each time the ESP32 publishes an integer, this function runs and prints it.”

    #self.get_logger().info(...) : parameters : get_logger() : A built-in method of every ROS2 node. It gives you a logger object that handles message logging.
    #info() : Logs an informational message (like a normal print, but with timestamp and node name).
    #f'Received counter value: {msg.data}' : The actual text of the message — it uses an f-string to include the integer received from the ESP32.
    #Think: Same as Serial.println("Message received!"); in Arduino, but formatted for ROS2 logging.

def main():
    node = counter_subscriber()
    #Step 5 : Spin the node =>rclpy.spin(node) **Keeps the node running — continuously checks for new messages, timers, or other ROS events and calls callbacks automatically.**
    rclpy.spin(node)
    #“Keep my node alive and responsive to any incoming data.”
    #Step 6 : shutdown ROs2 =>rclpy.shutdown() **Clean exit**
    node.destroy_node() #Step 6.1: Cleans up memory and stops publishers/subscribers in this node.
    rclpy.shutdown() # Step 6.2: Shuts down the ROS2 system cleanly, releasing all resources.
    #Think: “Close the connection between my code and the ROS network.”

#step 7 : Run the main function
if __name__ == '__main__':
    main()
#It ensures that the code inside main() runs only when the script is executed directly (not imported by another file).
