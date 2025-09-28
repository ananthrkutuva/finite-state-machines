# Robot Behaviors and Finite State Machines Writeup

CompRobo FA 2025 - Ananth Kutuva & Cian Linehan

## Introduction and Overview

This project was mainly for us to get a feel for the various concepts of ROS2 and robot software development as a whole. We were able to implement 4 behaviors with a finite state machine as well. Our initial simple behaviors were teleop control and driving in a square. The advanced behavior we chose to implement was person following. We combined person following, driving in a square, and if bumped, spin in place in our finite state machine.

## Behaviors

### Teleoperation

#### Overview

This behavior creates a TeleOp Node which takes in input from the user in the form of their WASD keys and controls the movement of the robot.

**W** : Drive Forward\
**A** : Rotate Counterclockwise\
**S** : Drive Backward\
**D** : Rotate Clockwise\
**Other Key** : Halt Movement (Brake)\
**Ctrl + C** : E-stop

#### Code Structure

We implemented this behavior by polling for keyboard inputs in getKey() using sys, tty, termios, and select, taking in one keystroke at a time in a non-blocking fashion. Alongside, we had a method called direction() which took in the key entered and called our drive() method to command the robot to turn drive at 0.25 m/s or rad/s, or stop. Our Neato was subscribed to the cmd_vel topic and receive our messages. getKey() and direction() were both called 10 times per second in the run_loop() method by our timer and quickly reacted to key inputs, publishing a Twist message to the 'cmd_vel' topic through our velocity_publisher. 

### Drive in a Square

#### Overview

This behavior creates a DriveSquare Node which commands the robot to drive in a square path two times.

#### Code Structure

This behavior used a similar method as teleoperation, publishing a Twist message to the 'cmd_vel' topic. We initially created a drive_forward() and turn_left() method and ran these consecutively 8 times to trace out the square path twice. Implementing drive_forward(), we commanded the robot to drive at 0.1 m/s using our drive() method, telling the program to sleep for (side length / velocity), in our case 5 seconds. Implementing turn_left(), we ran the same calculation for a 90 degree turn. 90 degrees is equivalent to pi/2 radians, turning at 0.3 rad/sec, we told the program to sleep for (radians / angular velocity), coming out to approximately 5.23 seconds.

### Person Following

#### Overview

This behavior creates a PersonFollowing Node which uses the laser scanning data to find the closes point cloud around the robot and drive toward it.

#### Code Structure

