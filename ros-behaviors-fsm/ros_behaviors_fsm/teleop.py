"""This ROS node allows the user to teleoperate the robot."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

"""Uses the following packages to take in keyboard input."""
import tty
import select
import sys
import termios


class TeleOp(Node):
    """
    This class should make the robot drive in different directions unless the bump sensor is pressed. If so, it stops in place.

    Publishers Needed:
        - Twist cmd_vel message: Commands wheel velocity in the linear and angular directions
    """

    def __init__(self):
        """Initializes the TeleOp Node with no inputs."""
        super().__init__("teleop")
        self.timer_period = 0.1

        # Stores the current key pressed
        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)

        """Creates a timer that runs every 0.1s"""
        self.timer = self.create_timer(self.timer_period, self.run_loop)

        """
        Create a publisher for controlling the motors.
        Will publish a message to the "cmd_vel" topic once a message is received.
        """
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def getKey(self):
        """
        Uses raw input through terminal attributes to take in keyboard input and control the robot.

        Returns:
            None
        """

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def direction(self):
        """
        Takes in a certain key input and commands the robot to drive in that direction.

            W - Forward
            A - Left
            S - Backward
            D - Right
            Crtl + C x2 - E-Stop
            Any Other Key - Halt Movement (Brake)

        Returns:
            None
        """

        if self.key_pressed == "\x03":
            rclpy.shutdown()
            return
        if self.key_pressed == "w":
            self.drive(0.25, 0.0)
        elif self.key_pressed == "a":
            self.drive(0.0, 0.25)
        elif self.key_pressed == "s":
            self.drive(-0.25, 0.0)
        elif self.key_pressed == "d":
            self.drive(0.0, -0.25)
        else:
            self.drive(0.0, 0.0)
            self.key_pressed = None

    def drive(self, linear, angular):
        """
        Drive with the inputted linear and angular velocity.

        Args:
            linear (float64): the inputted linear velocity
            angular (float64): the inputted angular velocity

        Returns:
            None
        """

        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.velocity_publisher.publish(vel)

    def run_loop(self):
        """Is called by the timer every 0.1 seconds and runs the getKey() and direction() methods."""

        self.getKey()
        self.direction()


def main(args=None):
    """Initializes the node, run it, and cleanup on shut down."""

    rclpy.init(args=args)
    node = TeleOp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
