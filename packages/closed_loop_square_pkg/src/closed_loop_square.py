#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, Pose2DStamped
import math

class ClosedLoopSquare:
    def __init__(self):
        self.TICKS_PER_METRE = 300
        self.WHEEL_TRACK = 0.1

        self.cmd_msg = Twist2DStamped()

        self.start_left_ticks = None
        self.start_right_ticks = None
        self.curr_left_ticks = None
        self.curr_right_ticks = None
        
        # State machine variables
        self.state = 'idle'
        self.ticks_calibrated = False  # Does start ticks == curr ticks?
        self.moving = True # Start moving when the node starts
        self.side_count = 0 # Number of sides completed in the square
        
        # Parameters
        self.target_distance = 1.0
        self.target_angle = math.pi / 2
        self.speed = 0.2
        self.angular_speed = 2.0
        
        # Pose callback variables
        self.start_pose = None
        self.end_pose = None
        self.pose_received = False

        rospy.init_node('closed_loop_square_node', anonymous=True)
        
        self.pub = rospy.Publisher(
            '/duckiebot1/car_cmd_switch_node/cmd', 
            Twist2DStamped, 
            queue_size=1
        )
        
        rospy.Subscriber(
            '/duckiebot1/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_callback,
            queue_size=1
        )
        
        rospy.Subscriber(
            '/duckiebot1/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_callback,
            queue_size=1
        )
        
        rospy.Subscriber(
            '/duckiebot1/velocity_to_pose_node/pose',
            Pose2DStamped,
            self.pose_callback,
            queue_size=1
        )
        
    # Encoder tick callbacks
    def left_callback(self, msg):
        rospy.loginfo_once(f'Left encoder resolution: {msg.resolution}')
        rospy.loginfo_once(f"Left encoder type: {msg.type}")
        self.curr_left_ticks = msg.data
    
    def right_callback(self, msg):
        rospy.loginfo_once(f'Right encoder resolution: {msg.resolution}')
        rospy.loginfo_once(f'Right encoder type: {msg.type}')
        self.curr_right_ticks = msg.data
    
    # Pose callback
    def pose_callback(self, msg):
        if not self.pose_received:
            self.start_pose = msg
            self.pose_received = True
        else:
            self.end_pose = msg
    
    # Helper functions
    def stop_robot(self):
        self.cmd_msg.v = 0
        self.cmd_msg.omega = 0
    
    def calibrate_start_ticks(self):
        self.start_left_ticks = self.curr_left_ticks
        self.start_right_ticks = self.curr_right_ticks
        self.ticks_calibrated = True
    
    def compute_delta_ticks(self):
        delta_left_ticks = self.curr_left_ticks - self.start_left_ticks
        delta_right_ticks = self.curr_right_ticks - self.start_right_ticks
        return delta_left_ticks, delta_right_ticks
        
    def straight_positive_displacement_metres(self):
        delta_left_ticks, delta_right_ticks = self.compute_delta_ticks()
        # https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
        avg_delta_ticks = (delta_left_ticks + delta_right_ticks) / 2.0
        return abs((avg_delta_ticks / self.TICKS_PER_METRE))

    def rotation_radians(self):
        delta_left_ticks, delta_right_ticks = self.compute_delta_ticks()
        # https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
        # rosparam get /duckiebot1/kinematics_node/baseline = 0.1 (w)
        difference_delta_ticks = delta_right_ticks - delta_left_ticks
        return (difference_delta_ticks / self.TICKS_PER_METRE * self.WHEEL_TRACK)
        
    def go_straight(self, distance, speed):
        if not self.ticks_calibrated:
            self.calibrate_start_ticks()
        
        # If distance is less than target distance, move forward
        if self.straight_positive_displacement_metres() < abs(distance):
            self.cmd_msg.v = speed if distance > 0 else -speed
            self.cmd_msg.omega = 0
            return False
        else:
            return True
    
    def go_rotate(self, angle, angular_speed):
        if not self.ticks_calibrated:
            self.calibrate_start_ticks()
        
        if abs(self.rotation_radians()) < abs(angle):
            self.cmd_msg.v = 0
            self.cmd_msg.omega = angular_speed if angle > 0 else -angular_speed
            return False
        else:
            return True
        
    # State machine logic:
    # 1. Start in 'idle' state, wait for moving to be True (which it is by default)
    # 2. Transition to 'straight' state, move forward until target distance is reached
    # 3. Transition to 'rotate' state, rotate until target angle is reached
    # 4. Repeat 2-3 until 4 sides are completed, then transition back to 'idle'
    def closed_loop_square(self):
        if self.curr_left_ticks is None or self.curr_right_ticks is None:
            return

        if self.state == 'idle':
            self.stop_robot()

            if self.moving:
                self.state = 'straight'
                self.ticks_calibrated = False
        elif self.state == 'straight':
            done = self.go_straight(self.target_distance, self.speed)  # 1.0, 0.2
            
            if done:
                self.stop_robot()
                self.state = 'rotate'
                self.ticks_calibrated = False
        elif self.state == 'rotate':
            done = self.go_rotate(self.target_angle, self.angular_speed) # 90deg, 2.0

            if done:
                delta_left_ticks, delta_right_ticks = self.compute_delta_ticks()
                difference_delta_ticks = delta_right_ticks - delta_left_ticks
                
                rospy.loginfo(
                    f'90deg rotation used tick difference: {difference_delta_ticks} ticks'
                )

                self.stop_robot()
                self.side_count += 1
                self.ticks_calibrated = False

                if self.side_count >= 4:
                    self.state = 'idle'
                    self.moving = False
                    
                    rospy.loginfo(
                        f'START pose: ',
                        f'x={self.start_pose.x} ',
                        f'y={self.start_pose.y} ',
                        f'theta={self.start_pose.theta}'
                    )
                    
                    rospy.loginfo(
                        f'END pose: ',
                        f'x={self.end_pose.x} ',
                        f'y={self.end_pose.y} ',
                        f'theta={self.end_pose.theta}'
                    )
                else:
                    self.state = 'straight'

        self.pub.publish(self.cmd_msg)
            
    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.closed_loop_square()
            rate.sleep()
            
if __name__ == '__main__':
    try:
        state_machine = ClosedLoopSquare()
        state_machine.run()
    except rospy.ROSInterruptException:
        pass
