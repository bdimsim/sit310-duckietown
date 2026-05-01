#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
# from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):

        self.default_omega = 0.2 # Minimum turning speed
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        rospy.loginfo('target follower node initialised!')

        # Shutdown signal (ctrl + c) triggers shutdown callback
        rospy.on_shutdown(self.shutdown_callback)
        
        # Subscribe to the robot's movement command centre topic
        self.cmd_vel_pub = rospy.Publisher(
            '/duckiebot1/car_cmd_switch_node/cmd', 
            Twist2DStamped, 
            queue_size=1
        )

        rospy.loginfo(
            'Subscribed to the topic: /duckiebot1/car_cmd_switch_node/cmd'
        )

        # Subscribe to the robot's apriltag node that contains information
        # about the tag's translation (position) and rotation (orientation)
        rospy.Subscriber(
            '/duckiebot1/apriltag_detector_node/detections', 
            AprilTagDetectionArray, 
            self.tag_callback, 
            queue_size=1
        )

        rospy.loginfo(
            'Subscribe to the topic: /duckiebot1/apriltag_detector_node/detections'
        )

        # Keep node alive and listen for message callbacks
        rospy.spin()

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node is shut down with a zero 
    # velocity command
    def shutdown_callback(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity command to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        ################################################################
        #                   Seek an object feature                     #
        ################################################################
        # If no tag is detected, publish in-place rotation commands until
        # the robot detects a tag
        if len(detections) == 0:
            cmd_msg.omega = self.default_omega
            self.cmd_vel_pub.publish(cmd_msg)
            return

        ################################################################
        #               Look at the object feature                     #
        ################################################################
        # The robot should keep looking at the object even if it is moving

        # x, y, z coordinates of the apriltag (metric)
        # x: Horizontal displacement. Positive is right, negative is left
        # y: Vertical displacement. Positive is down, negative is up
        # z: Distance from the camera. Always positive. Larger is further away
        # Note: The center of the tag is the origin (0,0,0)
        x = detections[0].transform.translation.x
        y = detections[0].transform.translation.y
        z = detections[0].transform.translation.z

        rospy.loginfo("x,y,z: %f %f %f", x, y, z)

        # negative x, sign is left ---> Need to rotate left  ---> Need positive omega
        # positive x, sign is right ---> Need to rotate right ---> Need negative omega
        # We need to scale x with a constant to control rotation speed and direction
        k_x = 0.8
        deadband = 0.05 # In metres
        omega = -k_x * x

        # Neutral zone where close enough is good enough (to stop jitters)
        if (abs(x) < deadband):
            omega = 0.0

        # Publish velocity command
        cmd_msg.omega = max(min(omega, self.default_omega), -self.default_omega)
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
