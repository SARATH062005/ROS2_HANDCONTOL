import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandGestureRobot(Node):
    def __init__(self):
        super().__init__('hand_gesture_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)  # Open webcam
        self.hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

        self.last_command = None  # Store last command to prevent flickering

    def count_fingers(self, hand_landmarks):
        """Counts the number of extended fingers"""

        fingers_up = 0
        finger_tips = [
            mp_hands.HandLandmark.INDEX_FINGER_TIP,
            mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            mp_hands.HandLandmark.RING_FINGER_TIP,
            mp_hands.HandLandmark.PINKY_TIP
        ]
        finger_bases = [
            mp_hands.HandLandmark.INDEX_FINGER_MCP,
            mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
            mp_hands.HandLandmark.RING_FINGER_MCP,
            mp_hands.HandLandmark.PINKY_MCP
        ]

        # Check if fingers are extended
        for tip, base in zip(finger_tips, finger_bases):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[base].y:  # Tip above MCP = Finger Up
                fingers_up += 1

        return fingers_up

    def detect_gesture(self, hand_landmarks):
        """Detects gesture based on number of fingers up"""

        fingers_up = self.count_fingers(hand_landmarks)

        if fingers_up == 0:
            return "STOP"
        elif fingers_up == 1:
            return "FORWARD"
        elif fingers_up == 2:
            return "BACKWARD"
        elif fingers_up == 3:
            return "LEFT"
        elif fingers_up == 4:
            return "RIGHT"
        else:
            return "STOP"  # Default case (Stop)

    def process_frame(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Convert to RGB and process
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(frame_rgb)

            twist_msg = Twist()
            detected_command = "STOP"  # Default to stop

            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    # Detect gesture based on finger count
                    detected_command = self.detect_gesture(hand_landmarks)

            # Prevent flickering (Only send command if changed)
            if detected_command != self.last_command:
                print(f"Gesture Detected: {detected_command}")  # Print to terminal
                self.get_logger().info(f"Command: {detected_command}")  # ROS 2 logging

                if detected_command == "FORWARD":
                    twist_msg.linear.x = 0.5
                elif detected_command == "BACKWARD":
                    twist_msg.linear.x = -0.5
                elif detected_command == "LEFT":
                    twist_msg.angular.z = 0.5
                elif detected_command == "RIGHT":
                    twist_msg.angular.z = -0.5
                else:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0  # Stop

                # Publish command to robot
                self.publisher_.publish(twist_msg)
                self.last_command = detected_command  # Update last command

            cv2.imshow("Hand Gesture Control", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def cleanup(self):
        """Release camera and close OpenCV windows."""
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = HandGestureRobot()
    try:
        node.process_frame()
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
