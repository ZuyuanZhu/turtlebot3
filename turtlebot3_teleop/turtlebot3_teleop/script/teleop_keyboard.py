#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

import math

msg = """
The robot will move along a predefined path to trigger the telejump.
The robot's main movements:
1. Turn 180 degree around.
2. Move forward with speed 0.26m/s until reaching a specific point.
3. Turn right until the robot's head direction is parallel to the wall.
4. Move forward with speed 0.26m/s until robot telejump to the home position. 

CTRL-C to quit
"""

FORWARD_SPEED = 0.26  # m/s
TURN_SPEED = 0.6  # rad/s for angular velocity
angle_tolerance =0.01 # rad for  controlling error

class AutoMoveBot(Node):
    def __init__(self):
        super().__init__('turtlebot3_auto')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_pose = None
        self.is_turning_180 = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def turn_around(self):
        # if self.is_turning:
        #     return  # Avoid re-entering if already turning

        if not self.current_pose:
            self.get_logger().info('Current pose not available.')
            return

        initial_yaw = self.get_yaw_from_pose(self.current_pose)
        target_yaw = initial_yaw + math.pi  # Add 180 degrees in radians

        # Normalize target_yaw to be within [-pi, pi]
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        # Debug information
        self.get_logger().info(f'Initial yaw: {initial_yaw:.2f}, Target yaw: {target_yaw:.2f}')

        twist = Twist()
        twist.angular.z = TURN_SPEED
        self.pub.publish(twist)

        def angle_difference(target, current):
            diff = target - current
            # Normalize to [-pi, pi]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            return abs(diff)

        while rclpy.ok():
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            diff = angle_difference(target_yaw, current_yaw)

            # Debug information
            self.get_logger().info(f'Current yaw: {current_yaw:.2f}, Angle difference: {diff:.2f}')

            if diff < angle_tolerance:  # Adjust tolerance as needed
                twist.angular.z = 0.0  # Stop turning
                self.pub.publish(twist)
                # Debug information
                self.get_logger().info('Reached target yaw. Stopping rotation.')
                self.is_turning_180 = True
                break

            rclpy.spin_once(self, timeout_sec=0.1)

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=duration)
        twist.linear.x = 0.0  # Stop
        self.pub.publish(twist)

    def run(self):
        while rclpy.ok():
            if not self.is_turning_180:
                self.turn_around()  # Example action
            # Add other movements here, with checks for orientation/position as needed
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    auto_move_bot = AutoMoveBot()
    
    try:
        auto_move_bot.run()
    except KeyboardInterrupt:
        auto_move_bot.get_logger().info('Node was stopped manually.')
    finally:
        auto_move_bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
