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
TURN_SPEED = 0.2  # rad/s for angular velocity
TURN_SPEED_SLOW = 0.1
angle_tolerance =0.01 # rad for  controlling error
ROTATE_THRETHOLD = 0.0001  # threshold, robot's stop rotating

def angle_difference(target, current):
            diff = target - current
            # Normalize to [-pi, pi]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            return abs(diff)

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

        while rclpy.ok():
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            diff = angle_difference(target_yaw, current_yaw)

            # Debug information
            self.get_logger().info(f'Current yaw: {current_yaw:.2f}, Angle difference: {diff:.2f}')

            if diff < TURN_SPEED * 2:  # slow down
                twist.angular.z = TURN_SPEED_SLOW
                self.pub.publish(twist)

            if diff < angle_tolerance:  # Adjust tolerance as needed
                twist.angular.z = 0.0  # Stop turning
                self.pub.publish(twist)
                # Debug information
                self.get_logger().info('Reached target yaw. Stopping rotation.')
                self.is_turning_180 = True
                break

            rclpy.spin_once(self, timeout_sec=0.1)

    def is_rotating(self):
        twist = Twist()
        if twist.angular.z > ROTATE_THRETHOLD:
            self.get_logger().info(f'Still rotating: {twist.angular.z:.2f}, Threthold: {ROTATE_THRETHOLD:.5f}')
            return True
        else:
            self.get_logger().info(f'Stop rotating: {twist.angular.z:.2f}, Threthold: {ROTATE_THRETHOLD:.5f}')
            return False      

    def move_forward_to_position(self, target_x, target_y, tolerance=0.1):
        if not self.current_pose:
            self.get_logger().info('Current pose not available.')
            return

        # Calculate the initial distance to the target
        initial_position = self.current_pose.position
        distance_to_target = math.sqrt(
            (target_x - initial_position.x) ** 2 + 
            (target_y - initial_position.y) ** 2)

        self.get_logger().info(f'Initial distance to target: {distance_to_target:.2f} meters')

        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.pub.publish(twist)

        while rclpy.ok():
            current_position = self.current_pose.position
            distance_to_target = math.sqrt(
                (target_x - current_position.x) ** 2 + 
                (target_y - current_position.y) ** 2)

            # Debug information
            self.get_logger().info(f'Current distance to target: {distance_to_target:.2f} meters')

            if distance_to_target < tolerance:  # Stop if within tolerance
                twist.linear.x = 0.0  # Stop moving forward
                self.pub.publish(twist)
                self.get_logger().info('Reached target position. Stopping.')
                break

            rclpy.spin_once(self, timeout_sec=0.1)


    def run(self):
        while rclpy.ok():
            if not self.is_turning_180:
                self.turn_around()  # Turn 180 degrees
            else:
                # empty run for 1 s   TODO
                # Move to the target position after turning around
                if not self.is_rotating():
                    # empty run for 1s
                    current_yaw = self.get_yaw_from_pose(self.current_pose)
                    self.get_logger().info(f'Current yaw: {current_yaw:.2f}')

                    self.move_forward_to_position(-8.26, 0.47)
                    break  # Exit the loop once the target position is reached
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
