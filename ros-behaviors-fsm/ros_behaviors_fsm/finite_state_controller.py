"""
Finite State Machine Controller
~~~~~~~~~~~~~~~~~~~~~~~
Drive the neato in a square until it is bumped. When bumped, person follow (likely whatever bumped it).

"""

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from time import sleep

class FSMControllerNode(Node):
    """
    Class that runs drive square until the neato is bumped, in which case it will run 
    person following
    """

    def __init__(self):
        """
        Initializes the class
        """
        super().__init__("FSMControllerNode")

        #initializing person followng values
        self.item_error = 0.0
        self.avg_x = 0.0
        self.avg_y = 0.0
        self.item_distance = 0.0
        self.bumped = False
        self.state = "square"

        #create the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        #wheel vel publisher
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        #lidar and nump sensors subscribers
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.get_item_error, 10)
        self.bump_sub = self.create_subscription(Bump, "bump", self.handle_bump, 10)

        #square driving
        self.square_stage = 0        # 0=forward, 1=turn
        self.stage_start_time = self.get_clock().now()

    def get_item_error(self, msg: LaserScan):
        """
        Handles laserscans from the subscriber
        """

        #initalize variables
        scans = np.array(msg.ranges)
        ranges = np.array(range(361))
        x = 0
        deg_to_rad = np.pi / 180

        #remove scans that are too far or return as zero
        while x < len(scans):
            current_scan = scans[x]
            if current_scan > 1.0 or current_scan <= 0:
                scans = np.delete(scans, x)
                ranges = np.delete(ranges, x)
            else:
                x += 1

        
        if len(scans) > 0:
            #change scans to cartesian
            x_coords = np.multiply(scans, np.cos(np.multiply(ranges, deg_to_rad)))
            y_coords = np.multiply(scans, np.sin(np.multiply(ranges, deg_to_rad)))

            # averages the scan coordinates to find the center of the scan (the person)
            self.avg_x = np.average(x_coords)
            self.avg_y = np.average(y_coords)

            # get dist and angle to person
            self.item_distance = np.sqrt(self.avg_x**2 + self.avg_y**2)
            theta = np.arctan2(self.avg_y, self.avg_x)
            self.item_error = theta

        else:
            self.item_error = 0.0

    def handle_bump(self, msg: Bump):
        """
        handles bump sensor
        """
        # check to see if bumped
        self.bumped = (msg.left_front or msg.right_front or msg.left_side or msg.right_side)
        
        # toggle state on any bump
        if self.bumped:
            if self.state == "square":
                self.state = "follower"
            else:
                self.state = "square"
            
            # reset square FSM when switching back to square
            if self.state == "square":
                self.square_stage = 0
                self.stage_start_time = self.get_clock().now()

    def run_loop(self):
        """
        loop that runs every timer incrimint
        """
        vel = Twist()

        if self.state == "follower":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            # now handle following
            if abs(self.item_error) <= 0.5:
                vel.linear.x = 0.2
            vel.angular.z = 0.75 * self.item_error


        elif self.state == "square":
            #manually writing drive in a square
            #the sleep() approach used in our drive_square doesn't work
            #because it cannot read bumper sensor inputs.
            now = self.get_clock().now()
            elapsed = (now - self.stage_start_time).nanoseconds * 1e-9

            if self.square_stage == 0:   # forward
                vel.linear.x = 0.1
                vel.angular.z = 0.0
                if elapsed > 2.0:
                    self.square_stage = 1
                    self.stage_start_time = now

            elif self.square_stage == 1:  # turn left
                vel.linear.x = 0.0
                vel.angular.z = 0.3
                if elapsed > (np.pi/2)/0.3:  # time to turn 90 deg
                    self.square_stage = 0
                    self.stage_start_time = now

        self.vel_pub.publish(vel)

def main(args=None):
    """
    main func
    """
    rclpy.init(args=args)
    node = FSMControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()