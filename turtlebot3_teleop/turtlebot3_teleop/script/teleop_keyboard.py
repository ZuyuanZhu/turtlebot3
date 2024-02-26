#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
The robot will move along a predefined path to trigger the telejump.
The robot's main movements:
1. Turn 180 degree around.
2. Move forward with speed 0.26m/s until reaching point (x=-8.6, y=1.14)
3. Turn right with angular speed (?) until the robot's head direction is parallel to the wall.
4. Move forward with speed 0.26m/s until robot telejump to the home position. 

CTRL-C to quit
"""

# Constants
FORWARD_SPEED = 0.26  # m/s
TURN_SPEED = 0.6  # rad/s for angular velocity
TURN_180_DURATION = 3.14 / TURN_SPEED  # Time to turn 180 degrees in seconds
TURN_TO_YAW_DURATION = 1.02  # Approximate time to reach desired yaw with TURN_SPEED

class AutoMoveBot(Node):
    def __init__(self):
        super().__init__('turtlebot3_auto')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=duration)
        twist.linear.x = 0.0  # Stop
        self.pub.publish(twist)

    def turn_around(self):
        twist = Twist()
        twist.angular.z = TURN_SPEED
        self.pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=TURN_180_DURATION)
        twist.angular.z = 0.0  # Stop
        self.pub.publish(twist)

    def turn_right_to_yaw(self):
        twist = Twist()
        twist.angular.z = -TURN_SPEED  # Assuming right turn requires negative speed
        self.pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=TURN_TO_YAW_DURATION)
        twist.angular.z = 0.0  # Stop
        self.pub.publish(twist)

    def move_forward_until_condition(self):
        print(msg)
        self.turn_around()  # Step 1: Turn 180 degree
        self.move_forward(TURN_180_DURATION)  # Using the turn duration for forward movement as an example
        
        # Here you would implement the actual movement and checking logic
        # For demonstration, we're moving forward for a predefined duration
        # Replace this with your condition for stopping
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.pub.publish(twist)
        start_time = self.get_clock().now()
        while rclpy.ok():
            current_time = self.get_clock().now()
            elapsed_time = current_time - start_time
            if elapsed_time.nanoseconds > TURN_TO_YAW_DURATION * 1e9:  # This is just a placeholder condition
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.linear.x = 0.0  # Stop
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    auto_move_bot = AutoMoveBot()
    
    try:
        auto_move_bot.move_forward_until_condition()
    except KeyboardInterrupt:
        auto_move_bot.get_logger().info('Node was stopped manually.')
    finally:
        auto_move_bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
