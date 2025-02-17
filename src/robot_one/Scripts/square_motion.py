#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMotion(Node):
    def __init__(self):
        super().__init__('square_motion')
        # Create a publisher to the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Ensure ROS 2 communication is ready

    def move_forward(self, duration=5.0, speed=0.5):
        """Move forward for a given duration."""
        msg = Twist()
        msg.linear.x = speed  # Set forward speed
        msg.angular.z = 0.0  # No angular velocity (no turning)
        self.publisher_.publish(msg)
        self.get_logger().info("Moving forward...")
        time.sleep(duration)
        self.stop_robot()

    def turn_left(self, duration=5.0, turn_speed=0.5):
        """Turn left for a given duration."""
        msg = Twist()
        msg.linear.x = 0.0  # No forward motion
        msg.angular.z = turn_speed  # Set angular velocity for turning
        self.publisher_.publish(msg)
        self.get_logger().info("Turning left...")
        time.sleep(duration)
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot."""
        msg = Twist()  # Zero velocity
        self.publisher_.publish(msg)
        time.sleep(1)  # Pause to stabilize

    def run_square(self):
        """Make the robot move in a square pattern."""
        for _ in range(4):  # Four sides of the square
            self.move_forward()
            self.turn_left()
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    # Wait for 5 seconds before starting the program
    time.sleep(5)

    node = SquareMotion()
    node.run_square()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
