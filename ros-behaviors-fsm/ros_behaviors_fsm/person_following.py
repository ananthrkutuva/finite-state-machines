"""This node uses the laser scan measurement to check if there is an object in the robot's radius. If so, the robot will start to follow that object."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
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
    MINIMUM_SCAN_DISTANCE = 0.1
    MAXIMUM_SCAN_DISTANCE = 2.0

    def __init__(self):
        """Initializing the PersonFollowing Node with no inputs."""
        super().__init__("person_following")

        # Setting up all the variables to be used
        self.timer_period = 0.1
        self.scanned_points = None
        self.angles = None
        self.needed_angle = 0.0
        self.avg_x_pos = 0.0
        self.avg_y_pos = 0.0
        self.obj_dist = 0.0

        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )
        self.visual_pub = self.create_publisher(
            Marker, "visualization_marker", self.create_marker, 10
        )

    def run_loop(self):
        """The main loop for checking the error between the angle of the closest object and the robot, turning toward it, and driving toward it."""
        if self.needed_angle > 0.5:
            self.linear_velocity = 0.0
        else:
            self.linear_velocity = 0.2

        # The angular velocity is proportional to the difference in angle between the Neato and the object
        self.angular_velocity = 0.75 * self.needed_angle
        self.drive(self.linear_velocity, self.angular_velocity)

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

        # Value to convert angles in degrees to radians
        degree_conversion = np.pi / 180

        # Creating helper array to remove values out of our bounds, helper assigns True to values that are in our range
        helper = (self.scanned_points > self.MINIMUM_SCAN_DISTANCE) & (
            self.scanned_points < self.MAXIMUM_SCAN_DISTANCE
        )

        # scanned_points and angles (now smaller) only contain values that are in our range
        self.scanned_points = self.scanned_points[helper]
        self.angles = self.angles[helper]

        # Runs the distance and angle calculation if there is a cluster of points in the threshold area (ty charlie)
        if len(self.scanned_points) > 0:

            # Converting our polar coordinates to cartesian coordinates
            self.x_coords = np.multiply(
                self.scanned_points, np.cos(np.multiply(self.angles, degree_conversion))
            )
            self.y_coords = np.multiply(
                self.scanned_points, np.sin(np.multiply(self.angles, degree_conversion))
            )

            # Finds the center of mass of the object by taking the average position of all the x and y coordinates
            self.avg_x_pos = np.average(self.x_coords)
            self.avg_y_pos = np.average(self.y_coords)

            # Using the given object position, calculates the distance and angle of it from the robot
            self.obj_dist = np.sqrt(self.avg_x_pos**2 + self.avg_y_pos**2)
            self.angle = np.arctan2(self.avg_y_pos, self.avg_x_pos)

            self.needed_angle = self.angle
        else:
            self.obj_dist = 100
            self.needed_angle = 0.0

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

    def create_marker(self):
        """Creates a marker for RViz2 to visualize where the center of mass of the object detected is."""
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.avg_x_pos
        marker.pose.position.y = self.avg_y_pos
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.g = 1.0

        self.visualizer_publisher.publish(marker)


def main(args=None):
    """Initializes the node, run it, and cleanup on shut down."""

    rclpy.init(args=args)
    node = PersonFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
