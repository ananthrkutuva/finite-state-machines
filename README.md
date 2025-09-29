# Robot Behaviors and Finite State Machines Writeup

CompRobo FA 2025 - Ananth Kutuva & Cian Linehan

## Introduction and Overview

This project was mainly for us to get a feel for the various concepts of ROS2 and robot software development as a whole. We were able to implement 4 behaviors with a finite state machine as well. Our initial simple behaviors were teleop control and driving in a square. The advanced behavior we chose to implement was person following. We combined driving in a square and person following, where the Neato drives in a square continuously, and if bumped, follow a person.

## Behaviors

### Teleoperation

#### Overview

This behavior creates a TeleOp Node which takes in input from the user in the form of their WASD keys and controls the movement of the Neato.

**W** : Drive Forward\
**A** : Rotate Counterclockwise\
**S** : Drive Backward\
**D** : Rotate Clockwise\
**Other Key** : Halt Movement (Brake)\
**Ctrl + C** : E-stop

#### Code Structure

We implemented this behavior by polling for keyboard inputs in getKey() using sys, tty, termios, and select, taking in one keystroke at a time in a non-blocking fashion. Alongside, we had a method called direction() which took in the key entered and called our drive() method to command the Neato to turn drive at 0.25 m/s or rad/s, or stop. Our Neato was subscribed to the cmd_vel topic and receive our messages. getKey() and direction() were both called 10 times per second in the run_loop() method by our timer and quickly reacted to key inputs, publishing a Twist message to the ```/cmd_vel``` topic through our velocity_publisher. 

### Drive in a Square

#### Overview

This behavior creates a DriveSquare Node which commands the Neato to drive in a square path two times.

#### Code Structure

This behavior used a similar method as teleoperation, publishing a Twist message to the ```/cmd_vel``` topic. We initially created a drive_forward() and turn_left() method and ran these consecutively 8 times to trace out the square path twice. Implementing drive_forward(), we commanded the Neato to drive at 0.1 m/s using our drive() method, telling the program to sleep for (side length / velocity), in our case 5 seconds. Implementing turn_left(), we ran the same calculation for a 90 degree turn. 90 degrees is equivalent to pi/2 radians, turning at 0.3 rad/sec, we told the program to sleep for (radians / angular velocity), coming out to approximately 5.23 seconds.

### Person Following

#### Overview

This behavior creates a PersonFollowing Node which uses the laser scanning data to find the closes point cloud around the Neato and drive toward it.

#### Code Structure

Our control code can be split up into 6 steps:

1. Receive data from the LaserScan message and ```/scan``` topic
2. Filter out the data points that are > 2 meters away and < 0.1 meters away from the Neato
3. Convert the data points from polar to cartesian coordinates
4. Find the center of the cluster of points closest to the Neato
5. Calculate the distance and angle from the Neato to the cluster
6. Turn and drive toward the cluster by publishing to ```/cmd_vel```

This behavior was much more involved compared to the other two as we needed to take information from the laser scan of the Neato and use that data to inform the Neato's movements. By subscribing to LaserScan messages on the ```/scan``` topic, we can use an array of points organized by degree values around the Neato to find a grouping of points that are within a certain radius of the Neato. We can then convert these points from polar coordinates (radius, theta) to cartesian (X, Y) coordinates to calculate the location of this cluster compared to the Neato. We use Pythagorean Theorem to calculate the distance from the Neato to the cluster as well as the angle between the object and the heading of the Neato. Using this needed_angle value, we can command the Neato to turn, with larger angles commanding a faster angular velocity.\

As the run_loop executes 10 times per second, the needed_angle value is constantly recalculated, stepping up or down the commanded angular velocity until the Neato is directly facing the cluster of points (the object).

## Finite State Machine
<img width="1444" height="718" alt="Screenshot from 2025-09-29 00-44-20" src="https://github.com/user-attachments/assets/8dd19a70-7f16-4b49-b90d-91e20fa0319e" />

### Driving in a Square + Person Following + Bump Sensing

