#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        rospy.init_node("turtlesim_straights_and_turns_node", anonymous=True)

        rospy.Subscriber("/turtle1/pose", Pose, self.update_pose_callback)
        rospy.Subscriber("/goal_coordinates", Point, self.update_goal_coordinates_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)

        self.velocity_publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        self.goal = Point() # Goal
        self.pose = Pose() # Turtle
        
        # Distance variables
        self.start_pose = None
        self.target_distance = 0

        # Angle variables
        self.prev_angle = None
        self.start_angle = None
        self.input_angle = 0
        self.total_angle_rotated = 0

        self.state = "IDLE"

        rospy.loginfo("Initialised node!")
        rospy.spin()

    def euclidean_distance(self):
        return ((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)**0.5

    def angle_to_goal(self):
        return math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

    def linear_vel(self, c=1.5):
        return c * self.euclidean_distance()

    def normalise_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi

    def angular_vel(self, target_angle=None, c=4):
        if target_angle is not None:
            return c * self.normalise_angle(target_angle - self.total_angle_rotated)
        else:
            return c * self.normalise_angle(self.angle_to_goal() - self.pose.theta)

    def update_pose_callback(self, msg):
        self.pose = msg

        if self.prev_angle is None:
            self.prev_angle = msg.theta
            self.pose.theta = msg.theta
        else:
            self.prev_angle = self.pose.theta
            self.pose.theta = msg.theta

    def goal_angle_callback(self, msg):
        self.input_angle = msg.data

        self.start_angle = self.pose.theta

        if self.input_angle == 0:
            self.state = "IDLE"
        else:
            self.state = "ROTATE_DIST"

    def goal_distance_callback(self, msg):
        d = msg.data

        if d == 0:
            self.state = "IDLE"
            return

        self.start_pose = self.pose
        self.target_distance = abs(d)

        self.move_direction = 1 if d > 0 else -1
        self.total_angle_rotated = 0
        self.prev_angle = self.pose.theta

        self.state = "MOVE_DIST"

    def update_goal_coordinates_callback(self, msg):
        self.goal = msg
        self.state = "ROTATE_TO_GOAL"

    def timer_callback(self, event):
        if self.prev_angle is None:
            return

        cmd_vel_msg = Twist()

        if self.state == "IDLE":
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0

        elif self.state == "ROTATE_TO_GOAL":
            relative_angle_to_goal = self.angle_to_goal() - self.pose.theta
            norm_relative_angle_to_goal = self.normalise_angle(relative_angle_to_goal)
            angle_tolerance = 0.05

            if abs(norm_relative_angle_to_goal) <= angle_tolerance:
                self.state = "MOVE"
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
            else:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = self.angular_vel()

        elif self.state == "MOVE_DIST":
            if self.start_pose is None:
                return

            distance_travelled = ((self.pose.x - self.start_pose.x)**2 + (self.pose.y - self.start_pose.y)**2)**0.5

            if distance_travelled >= self.target_distance:
                self.state = "IDLE"
                cmd_vel_msg.linear.x = 0
            else:
                cmd_vel_msg.linear.x = self.move_direction * self.linear_vel()
                cmd_vel_msg.angular.z = 0

        elif self.state == "ROTATE_DIST":
            angle_tolerance = 0.05

            current_angle = self.pose.theta
            relative_angle = self.normalise_angle(current_angle - self.start_angle)

            difference = self.normalise_angle(self.input_angle - relative_angle)

            if abs(difference) <= angle_tolerance:
                self.state = "IDLE"
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
            else:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = self.angular_vel(self.input_angle)

        elif self.state == "MOVE":
            distance_tolerance = 0.01
            if self.euclidean_distance() <= distance_tolerance:
                self.state = "IDLE"
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
            else:
                cmd_vel_msg.linear.x = self.linear_vel()
                cmd_vel_msg.angular.z = 0

        self.velocity_publisher.publish(cmd_vel_msg)

if __name__ == "__main__":
    try:
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass