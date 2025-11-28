import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2  # OpenCV for computer vision tasks
import mediapipe as mp  # Mediapipe for hand tracking

class led_control_pub(Node):
    def __init__(self):
        super().__init__('led_control_pub')
        self.publisher_ = self.create_publisher(Int32, 'led_control_cv', 10)
        self.get_logger().info("Publisher node started")

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6)
        self.cap = cv2.VideoCapture(0)

        self.squares = {
            "BLUE":   ((50, 50), (150, 150), (255, 0, 0)),
            "GREEN":  ((200, 50), (300, 150), (0, 255, 0)),
            "YELLOW": ((350, 50), (450, 150), (0, 255, 255)),
            "RED":    ((500, 50), (600, 150), (0, 0, 255)),
            "NONE":   ((500, 200), (600, 300), (200, 200, 200))  # None square  
        }
        self.selected_color = "NONE"  # Start with NONE
        self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz


    def publish_callback(self):
        msg = Int32()
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        fingertip_pos = None

        for name, (pt1, pt2, color) in self.squares.items():
            cv2.rectangle(frame, pt1, pt2, color, -1)
            cv2.putText(frame, name, (pt1[0]+10, pt1[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            #Detect if fingertip is in square

            if result.multi_hand_landmarks:
                for handLms in result.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(frame, handLms, self.mp_hands.HAND_CONNECTIONS)
                    x = int(handLms.landmark[8].x * w)
                    y = int(handLms.landmark[8].y * h)
                    fingertip_pos = (x, y)
                    cv2.circle(frame, fingertip_pos, 10, (255, 255, 255), -1)

            #update selected color if fingertip is in square
        if fingertip_pos:
            fx, fy = fingertip_pos
            for name, (pt1, pt2, color) in self.squares.items():
                if pt1[0] <= fx <= pt2[0] and pt1[1] <= fy <= pt2[1]:
                    self.selected_color = name  # store in variable

        msg.data = self.selected_color
        if self.selected_color == "BLUE":
            msg.data = 1
        elif self.selected_color == "GREEN":
            msg.data = 2
        elif self.selected_color == "YELLOW":
            msg.data = 3
        elif self.selected_color == "RED":
            msg.data = 4
        else:
            msg.data = 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        cv2.imshow("LED Color Selector", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = led_control_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
