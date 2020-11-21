#!/usr/bin/env python

# ROS libraries
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

import numpy

# Pre-defined global variables

_qwe = Vector3()
_asd = Vector3()
_pose = Vector3()
_cmd_vel = Twist()
_cam_pan = Float64()
_cam_tilt = Float64()

##############################################################
##############################################################
# Practical session P1.1
# The information retrived from the interface is stored in the
# variables _qwe, _asd, _pose
# Your task is produce the proper velocity output for the robot
# and the set the position of the camera.
# The velocity must be set in the variable _cmd_vel,
# the pan of the camera in the variable _cam_pan and the
# tilt of the camera in the variable _cam_tilt.

# If you require new global variables, add them here
# and declare them in the function process_data()

_max_abs_linear_vel = 3.0
_max_abs_angular_vel = 6.0
_vel_scale = 0.0
_init_x = 0.0
_init_y = 0.0
_new_command = False

_set_pan = 0.0
_set_tilt = 0.2

# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # If pressed key (q) _qwe.x = 1.0 ; (w) _qwe.y = 1.0; (e) _qwe.z = 1.0. Otherwise, they all equal to 0.0
    # If pressed key (a) _asd.x = 1.0 ; (s) _asd.y = 1.0; (d) _asd.z = 1.0. Otherwise, they all equal to 0.0
    # If click in the interface, _pose.x = vertical coordinate ;  _pose.y = horizontal coordinate. Otherwise, they all equal to -1.0
    # If click out of the interface, or the mouse leaves the interface, _pose.x = _pose.y = -1.0

    global _qwe, _asd, _pose, _cmd_vel

    global _max_abs_linear_vel, _max_abs_angular_vel
    global _vel_scale, _init_x, _init_y
    global _new_command
    global _set_pan, _set_tilt

    # Camera control
    if _qwe.z != 0:
        _set_pan = 0.785
    if _qwe.y != 0:
        _set_pan = 0.0
    if _qwe.x != 0:
        _set_pan = -0.785

    # Velocity control
    if _asd.z != 0:
        _vel_scale = min(_vel_scale+0.1,1)
    if _asd.y != 0:
        _vel_scale = 0.0
    if _asd.x != 0:
        _vel_scale = max(0,_vel_scale-0.1)

    # New velocity command

    if _new_command == False and _pose.x != -1 and _pose.y != -1:
        _init_x = _pose.x
        _init_y = _pose.y
        _new_command = True

    # Movement control
    diff_x = 0
    diff_y = 0
    if _new_command == True:
        if  _pose.x != -1 and _pose.y != -1:
            diff_x = _init_x - _pose.x
            if diff_x > 0:
                diff_x = min(diff_x, 150)
                diff_x = diff_x/150
            elif diff_x < 0:
                diff_x = max(diff_x, -150)
                diff_x = diff_x/150

            diff_y = _init_y - _pose.y
            if diff_y > 0:
                diff_y = min(diff_y, 150)
                diff_y = diff_y/150
            elif diff_y < 0:
                diff_y = max(diff_y, -150)
                diff_y = diff_y/150
        else:
            _new_command = False

    # Computing velocity

    vel_linear = _max_abs_linear_vel * _vel_scale * diff_x
    vel_angular = _max_abs_angular_vel * _vel_scale * diff_y

    if vel_linear < 0:
        vel_angular = vel_angular * -1

    # Outputs: replace the value 0.0 with your output
    # linear velocity, [-3.0,3.0] (+-1.5 m/s)
    _cmd_vel.linear.x = vel_linear
    # angular velocity, [-6.0,6.0] (+-3.0 rad/s)
    _cmd_vel.angular.z = vel_angular
    # camera pan, [-1.57,1.57] (rad)
    _cam_pan.data = _set_pan
    # camera tilt, [-1.57,1.57] (rad)
    _cam_tilt.data = _set_tilt

    # Cleaning keyboard input data after being processed, if you want
    # to keep it, you must store it in a global variable.
    # Other variables are not cleaded here.
    clean_keyboard_input()

##############################################################
##############################################################

# Cleans data after being processed
def clean_keyboard_input():
    global _qwe, _asd
    _qwe.x = _qwe.y = _qwe.z = 0
    _asd.x = _asd.y = _asd.z = 0

# Define callbacks and functions to process incoming messages
def qwe_callback(msg):
    global _qwe
    _qwe = msg

def asd_callback(msg):
    global _asd
    _asd = msg

def pose_callback(msg):
    global _pose
    _pose = msg

# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_teleoperation_node')

    # Subscribe to the topics and associate the corresponding callback functions
    sub_qwe = rospy.Subscriber('/robot/teleoperation/key_qwe/', Vector3, qwe_callback)
    sub_asd = rospy.Subscriber('/robot/teleoperation/key_asd/', Vector3, asd_callback)
    sub_pose = rospy.Subscriber('/robot/teleoperation/mouse_pose/', Vector3, pose_callback)

    # Publish messages
    pub_vel = rospy.Publisher('/robot/cmd_vel/', Twist, queue_size=10)
    pub_pan = rospy.Publisher('/robot/joint_pan_position_controller/command', Float64, queue_size = 1)
    pub_tilt = rospy.Publisher('/robot/joint_tilt_position_controller/command', Float64, queue_size = 1)

    rate=rospy.Rate(30)

    while not rospy.is_shutdown():

        process_data()
        pub_vel.publish(_cmd_vel)
        pub_pan.publish(_cam_pan)
        pub_tilt.publish(_cam_tilt)
        rate.sleep()
