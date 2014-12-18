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

def testPoseHomePose(arm=raven_constants.Arm.Left):
    homePose = tfx.pose([-0.08877309179873454, 0.055544224112048506, -0.11910100139152528],(0.9440970970460938, -0.090029831736168, -0.31471432570746993, 0.03911769345435281))
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm, False)
    rospy.sleep(1)
    ravenArm.start()


    print "Current Pose: ", ravenArm.getGripperPose()
    print "Homepose: ", homePose
    rospy.loginfo('Press enter to go to Home Pose')
    raw_input()
    # ravenArm.goToGripperPose(homePose)

    ravenArm.ravenController.stop()

if __name__ == '__main__':

    testPoseHomePose()