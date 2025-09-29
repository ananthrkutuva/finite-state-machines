# Robot Behaviors and Finite State Machines Writeup

CompRobo FA 2025 - Ananth Kutuva & Cian Linehan

## Introduction and Overview

This project was mainly for us to get a feel for the various concepts of ROS2 and robot software development as a whole. We were able to implement 3 behaviors with a finite state machine as well. Our initial simple behaviors were teleop control and driving in a square. The advanced behavior we chose to implement was person following. We combined driving in a square and person following, where the Neato drives in a square continuously, and if bumped, follow a person.

## How to Run

Open a terminal and ensure you are in a ROS2 workspace. Build the package, and source the environment:
```
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros-behaviors-fsm
source ~/ros2_ws/install/setup.bash
```
Then you can use ```ros2 run ros-behaviors-fsm "insert file name here"``` to run our 4 behaviors:

## Behaviors

### 1. Teleoperation

#### Overview

This behavior creates a TeleOp Node which takes in input from the user in the form of their WASD keys and controls the movement of the Neato. This behavior dealt with non-blocking code and fundamentally controlling the Neato.

**W** : Drive Forward\
**A** : Rotate Counterclockwise\
**S** : Drive Backward\
**D** : Rotate Clockwise\
**Other Key** : Halt Movement (Brake)\
**Ctrl + C** : E-stop

#### Code Structure

The overall structure is a single class, TeleOp, which inherits from the base Node class. We implemented this behavior by polling for keyboard inputs in getKey() using sys, tty, termios, and select, taking in one keystroke at a time in a non-blocking fashion. Alongside, we had a method called direction() which took in the key entered and called our drive() method to command the Neato to turn drive at 0.25 m/s or rad/s, or stop. Our Neato was subscribed to the cmd_vel topic and receive our messages. getKey() and direction() were both called 10 times per second in the run_loop() method by our timer and quickly reacted to key inputs, publishing a Twist message to the ```/cmd_vel``` topic through our velocity_publisher.

### 2. Drive in a Square

#### Overview

This behavior creates a DriveSquare Node which commands the Neato to drive in a square path two times. This behavior implemented a finite loop and math to calculate distance traveled as well as angle turned.

#### Code Structure

The single class, similar to teleoperation, called DriveSquare allows us to initialize all needed publishers and movement methods in one place. This behavior used a similar method as teleoperation, publishing a Twist message to the ```/cmd_vel``` topic. We initially created a drive_forward() and turn_left() method and ran these consecutively 8 times to trace out the square path twice. Implementing drive_forward(), we commanded the Neato to drive at 0.1 m/s using our drive() method, telling the program to sleep for (side length / velocity), in our case 5 seconds. Implementing turn_left(), we ran the same calculation for a 90 degree turn. 90 degrees is equivalent to pi/2 radians, turning at 0.3 rad/sec, we told the program to sleep for (radians / angular velocity), coming out to approximately 5.23 seconds.

### 3. Person Following

#### Overview

This behavior creates a PersonFollowing Node which uses the laser scanning data to find the closest point cloud around the Neato and drive toward it.

[Real Life Example](https://youtube.com/shorts/5FZWkgwF-qI?feature=share)\
[Rviz2 Example](https://youtu.be/bC2WSW05K24)

#### Code Structure

Our control code can be split up into 6 steps:

1. Receive data from the LaserScan message and ```/scan``` topic
2. Filter out the data points that are > 2 meters away and < 0.1 meters away from the Neato
3. Convert the data points from polar to cartesian coordinates
4. Find the center of the cluster of points closest to the Neato
5. Calculate the distance and angle from the Neato to the cluster
6. Turn and drive toward the cluster by publishing to ```/cmd_vel```

This behavior was much more involved compared to the other two as we needed to take information from the laser scan of the Neato and use that data to inform the Neato's movements. By subscribing to LaserScan messages on the ```/scan``` topic, we can use an array of points organized by degree values around the Neato to find a grouping of points that are within a certain radius of the Neato. We can then convert these points from polar coordinates (radius, theta) to cartesian (X, Y) coordinates to calculate the location of this cluster compared to the Neato. We use Pythagorean Theorem to calculate the distance from the Neato to the cluster as well as the angle between the object and the heading of the Neato. Using this needed_angle value, we can command the Neato to turn, with larger angles commanding a faster angular velocity.

As the run_loop executes 10 times per second, the needed_angle value is constantly recalculated, stepping up or down the commanded angular velocity until the Neato is directly facing the cluster of points (the object).

![IMG_9259](https://github.com/user-attachments/assets/32fcfa48-c974-4614-870f-bc77649c5a8b)

#### Figure 1. A Diagram of Our Point Thresholds, Point Cluster, and Angle Requirement

## Finite State Machine

#### Driving in a Square + Person Following + Bump Sensing

The main purpose of this assignment is to create a finite state machine, which combines the drive square node and the person follower node with a bump sensor state-switch. The Neato will drive in a square until the bumper is pressed. From there, it will person follow until the bumper is pressed again, where it will switch back to driving in a square.

<img width="1444" height="718" alt="Screenshot from 2025-09-29 00-44-20" src="https://github.com/user-attachments/assets/8dd19a70-7f16-4b49-b90d-91e20fa0319e" />

#### Figure 2. The RQT graph of our Finite State Machine

The above graph is a bit complicated and difficult to understand. As such, here is a simpler graph that captures the jist of the nodes and topics which make our finite state machine work:

![nodes_topics](https://github.com/user-attachments/assets/5f165d82-1edc-4ead-81b5-cc60394d3863)

#### Figure 3. Simpler Node/Topic graph of our Finite State Machine

### Code Structure

There are two states are run within our FSM node, driving in a square, and person following. The state starts driving in a square and is switched whenever the bumper is pressed. 

![states](https://github.com/user-attachments/assets/31206daa-2131-4595-9e53-99af71b7ee04)


#### Figure 4. State Transition Diagram

Finite State Machine Functions: The finite state machine is in charge of keeping track of the current active state (square or person following) and running the appropriate functions for the Neato to perform that behavior.

Square Functions: The sleep() implementation used in our drive_square.py code does not work for a finite state machine as during sleep, the neato doesn't recognize when the bumper is pressed. Instead, we simply coded the functionality of the square directly inside the run_loop() function. When in the "square" state, linear and angular velocity values are published so that the Neato drives in a square. While it does this, it also listens for the bumper to be pressed on the Neato. If they do, the state switches to person following and the Neato begins following a person, until the bumper is pressed again, and the state switches back to "square".

Person Following Functions: A reimplementation of the person follower behavior, with the added job of subscribing to the /bump topic and switching the state of the finite state machine from person following to square when the bump sensors are pressed.

### Challenges Faced
Ananth:

We faced several hurdles before starting this project while we were setting up our computers. The process of dual booting was very finicky and set us back the most during this project. Throughout the actual project, we faced a steep learning curve while getting a feel for the vocabulary used in ROS2 such as nodes, subscribers, publishers, topics, and messages. We were confused about where to begin with the project as we weren't able to get a full grasp of the in-class activites such as teleop or driving in a square before diving into the project. Once we got up to speed with the basic structure and understood how the various parts of our python files worked, we felt more comfortable tackling the advanced behaviors and finite state machine.

Cian:

Computational setups can cause the most challenges sometimes. Our #1 challenge was the computational setup. For most of the designated project time, both of our team members was experiencing many issues related to computational setup. As a result, we couldn't start the project until much later, shortening the amount of tiume we had to work on it.

Working with sleep(). Our drive_square.py code uses sleep() in order to drive the path of the square. However, when it came time to implement the drive_square into our finite state machine, we couldn't read bump sensor inputs from the neato while the Neato was driving in a square. As a result we had to effectively redo drive_square inside our finite state machine controller code to avoid using sleep(). The lesson learned from this is that, especially in systems that are taking constant readings from the environment, we should avoid the use of sleep() in our code.

### Improvements for the Future
Ananth:

If we had more time with this project, we would definitely have tried to optimize some of our scripts. For example, one bug that we weren't able to fix was that clicking Ctrl+C in our teleop code would need to be pressed twice before the script would exit. We weren't sure if this was because our key input method was truly non-blocking or if something else was causing this. We also originally attempted to include a bumper e-stop element in our teleop code but continously ran into an issue where the e-stop would only trigger after a key was pressed after the Neato ran into something, not immediately once it hit an object. We would like to expand our knowledge of PID as well as we were only able to experiment with proportional angular velocity control in our person follower. We want to explore using ROS parameters, threading, or more complex methods of control.

Cian:
One of the things that intrigued me when working on the person follower is experimenting with weighting closers lidar scans more heavily in the choice of direction. As the Neato person follows, it averages all the valid points, giving them equal weight. This became a small issue when near chairs and tables as the Neato would occasionally turn toward the chairs/tables instead of following the person. A potential fix for this is to weight closer measured points more heavily than further ones. This would ensure the Neato is following the person it is closest to and not get too distracted by objects in the distance.

Another interesting thing to do is if we did person following with obstacle avoidance/wall following so that the neato could maybe try follow a person through an imaginary labyrinth. This would pose an interesting challenge as rather than just driving toward the person, it would have to find the best way to get closer to the person. 

### Key Takeaways

In the future, I think a safe move would be to never use sleep(). Assignments will only get more complex and likely need constant sensor input that is lost when using sleep().

In terms of scoping, make account for when you need to do a computational setup and reserve plenty of time if you do. We should be fine for the coming projects, but for future work/classes.

Use print statements and topic echo for debugging. Being able to see what is actually going on in your code is good for debugging.

Even if you "divide and conquer" the work, make sure all team members understand as you go. Understanding when/as it is made can save much confusion and time later on.

Use what you're given. There is so much useful and necessary things on the CompRobo website and in the day activities. Scan through those for something relevant to your problem can be helpful. Also talk to your classmates. Many people encounter the same problem and talking can be helpful.

Some of our biggest takeaways included all the new concepts we have learned through this warmup project: setting up Nodes, publishing messages to topics, receiving sensor inputs through subscribers, logic control in each state are all super useful for our next projects.

## Team Member Contributions

#### Ananth
Implemented teleop driving and person following. Recorded ROS bags. Used RVIZ to visualize person following behavior. Simulation and Real Life Neato testing. Worked on write up.

#### Cian
Implemented square driving and finite state machine controller. Simulation and Real Life Neato testing. Worked on writeup.


