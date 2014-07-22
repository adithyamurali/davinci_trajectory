#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import numpy as np
import os
import math
import rospy
import tfx
import pickle

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
class Checkerboardtask():
    def __init__(self, arm=raven_constants.Arm.Right):
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("//dvrk_psm1/joint_position_cartesian", PoseStamped, self.pose_callback)        
        rospy.init_node('raven_commander',anonymous=True)
        # self.davinciArm = RavenArm(arm)
        rospy.sleep(1)
        # self.davinciArm.start()
        self.corners = {}
        self.joint = None
        self.pose = None
    def start(self, arm=raven_constants.Arm.Right):
        while True:
            if self.joint != None and self.pose != None:
                rospy.loginfo('Wait for teleoperator')
                raw_input()
                currPose = self.pose
                currJoint = self.joint
                print currPose, " XXXX ", currJoint
                rospy.loginfo('Input okay?')
                s = raw_input('-->')
                if s == "-1":
                    break
                elif s == "y":
                    rospy.loginfo('Entry #?')
                    s2 = raw_input('-->')
                    data = [currPose, currJoint]
                    print currPose
                    self.corners[s2] = data
                else:
                    continue

        pickle.dump(self.corners, open( "save.p", "wb" ) )

    def joint_callback(self, msg):
        self.joint = msg

    def pose_callback(self, msg):
        self.pose = msg

if __name__ == '__main__':
    task = Checkerboardtask()
    task.start()
    # task.davinciArm.ravenController.stop()