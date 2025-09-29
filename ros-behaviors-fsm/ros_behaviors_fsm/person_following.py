"""This node uses the laser scan measurement to check if there is an object in the robot's radius. If so, the robot will start to follow that object."""

import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from time import sleep
import numpy as np


class PersonFollowing(Node):
    """
    This Node inherits from the base Node class and commands the robot to follow an object in front of it.

    Publishers Required:
        - Twist cmd_vel message: Commands wheel velocity in the linear and angular directions

    Subscribers Required:
        - LaserScan scan message: Receives scan data from the lidar spinning
    """

    # Constants used for laser scanning
    MINIMUM_SCAN_DISTANCE = 0.01
    MAXIMUM_SCAN_DISTANCE = 4.0

    def __init__(self):
        """"""
        super().__init__("person_following")
        self.timer_period = 0.1
        self.scanned_points = None
        self.angles = None
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )

    def run_loop(self):
        """The main loop for checking what is near the robot and driving toward it."""
        print(self.scanned_points)
        print(self.angles)

    def process_scan(self, msg: LaserScan):
        """
        Takes in a scan of the nearby world and determines what the closest average point cloud is.

        Args:
            msg: A message received from LaserScan from the scan subscriber

        Returns:
            scanned_points: An array containing the various scanned points around the robot in meters
            angles: An array containing the angles that correspond with each scanned points from 0 to 360 degrees
        """

        # Reads the data from the laser scan subscriber
        self.scanned_points = np.array(msg.ranges)

        # Creates a corresponding angle array to plot the data against
        self.angles = np.array(range(361))

        # Creating helper array to remove values out of our bounds, helper assigns True to values that are in our range
        helper = (self.scanned_points > self.MINIMUM_SCAN_DISTANCE) & (
            self.scanned_points < self.MAXIMUM_SCAN_DISTANCE
        )

        # scanned_points and angles (now smaller) only contain values that are in our range
        self.scanned_points = self.scanned_points[helper]
        self.angles = self.angles[helper]

    def drive(self, linear, angular):
        """
        Drive with the inputted linear and angular velocity.

        Args:
            linear (float64): the inputted linear velocity
            angular (float64): the inputted angular velocity
        """

        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel)


def main(args=None):
    """Initializes the node, run it, and cleanup on shut down."""

    rclpy.init(args=args)
    node = PersonFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
