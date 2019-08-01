#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from goal_publisher.msg import PointArray
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist


class TurtleBot:
    def __init__(self):
        rospy.init_node('sd_171001_final')
        ###------------Subscribers-----------###
        # subscribing the position of the robot
        rospy.Subscriber('/gazebo/model_states',
                         ModelStates, self.position_clbk)
        # subscibing the coordinate of the goal
        rospy.Subscriber('/goals', PointArray, self.goals_clbk)
        rospy.Subscriber('/scan', LaserScan, self.laser_clbk)

        ###------------Publisher-----------###
        self.pub_vel = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=20)  # publishing velocity

        ###------------Initialization-----------###
        self.i = 0
        self.x = 0.0  # initial position
        self.y = 0.0
        self.yaw = 0  # initial yaw(angle of the robot)
        self.goals = [0, 0, 0]  # to hold the goals' positions
        self.goal_reached = [0, 0, 0]  # indicate how many goal reached
        self.regions = [0, 0, 0, 0, 0]  # divide regions

        # Array for storing 3 goals individually
        self.distance = [0.0, 0.0, 0.0]
        self.angle = [0.0, 0.0, 0.0]
        self.diff = [0.0, 0.0, 0.0]

        self.speed = Twist()  # for the movement of the robot
        self.d = 0.8  # nearest distance to obstacle

        self.rate = rospy.Rate(10)
        ###------------call back functions-----------###

    def goals_clbk(self, msg):  # callback for subscribing goals' coordinates
        self.goals[0] = msg.goals[0]
        self.goals[1] = msg.goals[1]
        self.goals[2] = msg.goals[2]

    def laser_clbk(self, msg):  # callback for laser scanner taken from mini_project
        self.regions[0] = min(msg.ranges[54:90])  # right
        self.regions[1] = min(msg.ranges[18:53])  # froRight
        self.regions[2] = min(msg.ranges[342:359] + msg.ranges[0:17])  # front
        self.regions[3] = min(msg.ranges[306:341])  # froLeft
        self.regions[4] = min(msg.ranges[270:305])  # left

    def position_clbk(self, msg):  # callback for subscribing robot position coordinates
        self.x = msg.pose[1].position.x
        self.y = msg.pose[1].position.y
        rotation = msg.pose[1].orientation
        (roll, pitch, self.yaw) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])  # subscribing yaw

        ###------------functions-----------###
    def dis_ang_diff_cal(self):  # calculating distance, angle, and angle differences
        self.distance[self.i] = math.sqrt(((self.goals[self.i].x-self.x)*(self.goals[self.i].x-self.x))+(
            (self.goals[self.i].y-self.y)*(self.goals[self.i].y-self.y)))
        self.angle[self.i] = math.atan2(
            (self.goals[self.i].y-self.y), (self.goals[self.i].x-self.x))
        self.diff[self.i] = self.angle[self.i] - self.yaw

    def obstacle_avoid(self):
        if self.regions[2] < 1:
            self.speed.linear.x = 0.3
            self.speed.angular.z = 1            #turn big when stuff is in front
            self.pub_vel.publish(self.speed)
            self.state = False

        elif self.regions[2] > 1:
            if self.regions[3] > 1:
                if self.regions[1] < 1:         #front right is blocked so turn a little bit left
                    self.speed.linear.x = 0.4
                    self.speed.angular.z = -0.3
                    self.pub_vel.publish(self.speed)
                    print"Adjusting to left"
                    self.state = False
                else:
                    self.state = True  
            elif self.regions[3] < 1:           #front left is blocked so turn a little bit right
                if self.regions[1] > 1: 
                    self.speed.linear.x = 0.4
                    self.speed.angular.z = 0.3
                    self.pub_vel.publish(self.speed)
                    print"Adjusting to right"
                    self.state = False
                else:
                    self.state = True 
        else:
            self.state = True 

    def rotate(self,s):
        self.speed.linear.x = 0.6
        self.speed.angular.z = s
        self.pub_vel.publish(self.speed)
        print self.diff[self.i], "- degree to the Goal", self.i+1

    def move(self,s):
        self.speed.linear.x = s
        self.speed.angular.z = 0
        self.pub_vel.publish(self.speed)
        print self.distance[self.i], "- distance to the Goal", self.i+1

    def navigate(self):
        while self.distance[self.i] >0.5:
            self.obstacle_avoid()
            if self.state == True:
                if abs(self.diff[self.i]) >0.05:
                    self.rotate(self.diff[self.i]/2)
                    self.dis_ang_diff_cal()           
                else:
                    self.move(1)
                    self.dis_ang_diff_cal()
                
    def goal_check(self):  # indicate goal reached
        if self.distance[self.i] <= 0.5:
            self.goal_reached[self.i] = 1
            print "Reached Goal", self.i+1, ":\n", self.goals[self.i], "in distance", self.distance[self.i]
            self.i = self.i+1

    def go2goal(self):
        print"AMR Final Project - Siyi Dai: sd-171001"
        while self.goal_reached != [1, 1, 1]:
            if self.goal_reached[self.i] == 1:
                rospy.sleep(1)
                self.dis_ang_diff_cal()
                print "Calculating the way to Goal", self.i+1, ":\n", self.goals[self.i]
                # print "Distance to goal", self.i+1, ":\n", self.distance[self.i]
                # print "Angle needed to go toward goal", self.i+1, ":\n", self.angle[self.i]
                # print "Angle difference left to goal", self.i+1, ":\n", self.diff[self.i]
                rospy.sleep(1)
                self.navigate()
                self.goal_check()
            else:
                rospy.sleep(1)
                self.dis_ang_diff_cal()
                print "Calculating the way to Goal", self.i+1, ":\n", self.goals[self.i]
                # print "Distance to goal", self.i+1, ":\n", self.distance[self.i]
                # print "Angle needed to go toward goal", self.i+1, ":\n", self.angle[self.i]
                # print "Angle difference left to goal", self.i+1, ":\n", self.diff[self.i]
                rospy.sleep(1)
                self.navigate()
                self.goal_check()
        print"All goals reached!"
        rospy.is_shutdown()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        my_turtlebot = TurtleBot()
        my_turtlebot.go2goal()
        my_turtlebot.rate.sleep()
        rospy.spin()
