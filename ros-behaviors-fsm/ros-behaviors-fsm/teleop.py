"""This ROS node allows the user to teleoperate the robot."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleOp(Node):
    """This teleoperation node inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the TeleOp Node with no inputs."""
        super().__init__('teleop')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)