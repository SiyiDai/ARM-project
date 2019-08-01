# AMR Final Project
This project is the final project for course Autonomous Mobile Robot. It contains a simple program that navigating the turtlebot to three different goals given by goals_publisher with obstacle avoidance. 

## Prerequisites
1. Linux
2. Install ROS
3. Install Dependent ROS Packages

## To start
`roslaunch sd_171001_prj start.launch`

## General Description

### Goal
Navigate the turtlebot to reach three different goals(in 0.5m distance) in 5mins with obstacle avoidance.

### Import
1. `rospy`
    - basic import
2. `math`
    - basic import
3. `sensor_msgs.msg` --> LaserScan
    - subscribing the laserscan
4. `gazebo_msgs.msg` --> ModelStates
    - subscribing the position of the robot
5. `goal_publisher.msg` --> PointArray
    - subscibing the coordinate of the goal
6. `tf.transformations` --> euler_from_quaternion
    - calculating the yaw
7. `geometry_msgs.msg` --> Twist
    - sublishing the velocity to turtlebot


### Subscriber:
- subscribing the position of the robot

`rospy.Subscriber('/gazebo/model_states', ModelStates, self.position_clbk)`

- subscibing the coordinate of the goal

`rospy.Subscriber('/goals', PointArray, self.goals_clbk)`

- subscibing the laserscan

`rospy.Subscriber('/scan', LaserScan, self.laser_clbk)`

### Publisher:
- publishing velocity

`self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=20)`

</br>
![nodes](presentation/rosgraph.png)
</br>

### Call-back functions:
- `goals_clbk(self, msg)`

In the initialization part, the `goals[]` has been defined as list to store the goals' coordinates.

- `laser_clbk(self, msg)`      

From laser data, with the division from mini_project into 5 areas
</br>
![regions](presentation/laser.png)
</br>

- `position_clbk(self, msg)`

In the initialization part, `x`, `y`, and `yaw` has been defined as float(since roll and pitch of the turtlebot does not influence the navigation here, they are not used). 
The `pose[0]` is refer to the ground, so here we choose `pose[1]` for subscribing the position and orientation of turtlebot. 

```
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
```
```
geometry_msgs/Quaternion.msg:
float64 x
float64 y
float64 z
float64 w
```
The `tf.euler_from_quaternion` has been used for converting quaternions to Euler angles. 


## Methods and Algorithm

### dis_ang_diff_cal
Use formula from math library, to calculate:

1. distance between turtlebot and the goals
2. angle between turtlebot and the goals
3. the diffrence between the angle needed and the current yaw turtlebot has

</br>
![cal](presentation/cal.png)
</br>

Code for the calculating:
  ```python
self.distance[self.i] = math.sqrt(((self.goals[self.i].x-self.x)*(self.goals[self.i].x-self.x))+((self.goals[self.i].y-self.y)*(self.goals[self.i].y-self.y)))
self.angle[self.i] = math.atan2((self.goals[self.i].y-self.y), (self.goals[self.i].x-self.x))
self.diff[self.i] = self.angle[self.i] - self.yaw
  ```

### obstacle_avoid
- If there is stuff near in the front region, turn right with low speed. 

</br>
![turn](presentation/turn.png)
</br>

- If there is no stuff near in the front region, but in front right, or in front left, turn slightly to the opposite with low speed.

</br>
![sli](presentation/slight.png)
</br>

- If there is no stuff near around, turn the state to ture, the navigate function can work.



### navigate
- While the distance between goal and turtlebot is longer then 0.5m, always detect is there obstacles around and try to avoid them.

- Once the state turns ture, which means the surrounding is clean, then always use the calculation above to compare the angle difference.

- If the diff is bigger than 0.05, always rotate half of the left value of diff to reduce the adjusting time.

### goal_check
Check whether the goal has been reached in 0.5m radias.
</br>
![reach](presentation/reach.png)
</br>

### go2goal
The main function: while not all the goals are reached, detect which goal has been reached by order, calculate the diff and do navigate and goal check.