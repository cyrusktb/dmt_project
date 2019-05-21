#!/usr/bin/env python

import rospy

from main_control.frame import Frame
from main_control.glove import Glove
from main_control.pose import Pose

if __name__ == '__main__':
    rospy.init_node("main_controller")

    frame = Frame()
    glove = Glove()
    pose = Pose()

    print(pose.get_pose())
    print(glove.get_servo_pos())
    print(frame.get_motor_pos())
    print(frame.get_motor_torques())
