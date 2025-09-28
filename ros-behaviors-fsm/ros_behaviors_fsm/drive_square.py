"""
Drive Square
~~~~~~~~~~~~~~~~~~~~~~~
Drive the robot on path along the edges of a square.

"""

import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import Twist
import math
from time import sleep


class DriveSquare(Node):
    """
    A class that implements a node to drive the robot in a square shape.
    """

    def __init__(self):
        super().__init__("drive_square")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def run_loop(self):
        """
        Main logic for driving the square.
        """

        self.drive(0.0, 0.0)
        sleep(1)

        for _ in range(8):  # Drive the square path twice
            self.drive_forward(0.5)
            self.turn_left()
        print("Finished")

    def drive(self, linear, angular):
        """
        Drive the robot given linear and angular velocities.

        Args:

        linear (float64): the linear velocity in m/s
        angular (float64): the angular velocity in rad/s
        """

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn_left(self):
        """
        Turn the robot left 90 degrees
        """
        angular_vel = 0.3
        self.drive(0.0, angular_vel)  # Start turning
        sleep(math.pi / angular_vel / 2)
        self.drive(0.0, 0.0)  # Stop turning

    def drive_forward(self, dist):
        """
        Drive the robot straight for a specified distance

        Args:

        dist (float64): distance to drive forward (only positive)
        """

        forward_vel = 0.1
        self.drive(forward_vel, 0.0)  # Start driving
        sleep(dist / forward_vel)
        self.drive(0.0, 0.0)  # Stop driving


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
