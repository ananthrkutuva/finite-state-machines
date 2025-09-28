"""This node uses the laser scan measurement to check if there is an object in front of it. If so, the robot will start to follow that object."""

import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from time import sleep

class PersonFollowing(Node):
    """This Node inherits from the base Node class and commands the robot to follow an object in front of it."""

    def __init__(self):
        super().__init__("person_following")
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def run_loop(self):
        """The main loop for checking what is near the robot and driving toward it."""
        self.get_laser_scan()

    def process_scan(self):
        """Takes in a scan of the nearby world and determines what the closes average point cloud is."""


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


