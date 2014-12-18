#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import numpy as np
import os
import math
import rospy
import tfx
import IPython

# rename so no conflict with raven_2_msgs.msg.Constants
from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

def testPoseHomePose():
    armLeft=raven_constants.Arm.Left
    armRight=raven_constants.Arm.Right


    homePoseLeft = tfx.pose([-0.03273119385791369, 0.040068107200195525, -0.11246711443109175],
        (-0.027563141345670266, -0.9919798717209513, 0.10806584652277598, 0.05948092261355594), frame='/one_remote_center_link')
    
    homePoseRight = tfx.pose([0.03440504833287911, 0.007756525635176232, -0.09716080022785732],
        (-0.9585515587214096, 0.24848103376519048, -0.09929863892839975, 0.09785635103919324), frame='/two_remote_center_link')


    rospy.init_node('raven_commander',anonymous=True)
    ravenArmLeft = RavenArm(armLeft, False)
    ravenArmRight = RavenArm(armRight, False)
    rospy.sleep(1)
    ravenArmLeft.start()
    rospy.sleep(1)
    ravenArmRight.start()

    startPoseLeft = ravenArmLeft.ravenController.currentPose
    startPoseRight = ravenArmRight.ravenController.currentPose
    print "Start Left Pose:"
    print repr(startPoseLeft)
    print "Start Right Pose:"
    print repr(startPoseRight)

    rospy.loginfo('Press enter to go to Home Pose')
    raw_input()
    ravenArmRight.goToGripperPose(homePoseRight)
    ravenArmLeft.goToGripperPose(homePoseLeft)
    ravenArmRight.setGripperPositionDaVinci(1)
    ravenArmLeft.setGripperPositionDaVinci(1)
    ravenArmRight.goToGripperPose(homePoseRight)
    ravenArmLeft.goToGripperPose(homePoseLeft)

    ravenArmRight.ravenController.stop()
    ravenArmLeft.ravenController.stop()

if __name__ == '__main__':

    testPoseHomePose()