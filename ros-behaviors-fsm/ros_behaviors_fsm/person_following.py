"""This node commands the robot to follow the closest object in its vicinity."""

import rclpy
from rclpy.node import Node
from threading import Thread
from geometry_msgs.msg import Twist
import math
from time import sleep

