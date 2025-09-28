"""This ROS node allows the user to teleoperate the robot."""

import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
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
    Subscribers Needed:
        - Bump to take in bump messages: Listens for bump sensor data
    """

    def __init__(self):
        """Initializes the TeleOp Node with no inputs."""
        super().__init__("teleop")
        self.timer_period = 0.1

        # Stores if the robot has been bumped
        self.bump_state = False

        # Stores the current key pressed
        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)

        """Creates a timer that runs every 0.1s"""
        self.timer = self.create_timer(timer_period, self.run_loop)

        """
        Create a subscriber to the bump sensor.
        Listens to the ROS networks and send messages over the "bump" topic name.
        """
        self.bump_subscriber = self.create_subscription(
            Bump, "bump", self.process_bump, 10
        )

        """
        Create a publisher for controlling the motors.
        Will publish a message to the "cmd_vel" topic once a message is received.
        """
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def getKey():
        """Uses raw input through terminal attributes to take in keyboard input and control the robot."""

        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key_pressed

    def bump_pressed(self, msg):
        """
        Callback for handling a bump sensor input.

        Args:
            msg (Bump): A Bump type message from the subscriber.
        """

        # Hitting any bumper causes the robot to e-stop
        self.bump_state = (
            msg.left_front == 1
            or msg.right_front == 1
            or msg.left_side == 1
            or msg.right_side == 1
        )

        if self.bump_state == True:
            rclpy.shutdown()

    def direction(self):
        """
        Takes in a certain key input and commands the robot to drive in that direction.

            W - Forward
            A - Left
            S - Backward
            D - Right
            Crtl + C - E-Stop
            Any Other Key - Halt Movement
        """

        if self.key == "w":
            self.drive(0.3, 0.0)
        elif self.key == "a":
            self.drive(0.0, 0.3)
        elif self.key == "s":
            self.drive(-0.3, 0.0)
        elif self.key == "d":
            self.drive(0.0, -0.3)
        elif self.key == "\x03":
            rclpy.shutdown()
        else:
            self.drive(0.0, 0.0)
            self.key = None
            
    def drive(self, linear, angular):
        """
        Drive with the inputted linear and angular velocity.

        Args:
            linear (float64): the inputted linear velocity in m/s
            angular (float64): the inputted angular velocity in radians/s
        """

        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.velocity_publisher.publish(vel)

    def run_loop(self):
        """Is called by the timer every 0.1 seconds and runs the getKey() and direction() methods."""

        self.getKey()
        self.direction()
        self.bump_pressed()

    def main(args=None):
        """Initializes the node, run it, and cleanup on shut down."""

        rclpy.init(args=args)
        node = TeleOp()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == "__main__":
        main()
