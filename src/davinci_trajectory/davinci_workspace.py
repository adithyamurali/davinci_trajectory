#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import numpy as np
import os
import math
import rospy
import tfx

# rename so no conflict with raven_2_msgs.msg.Constants
from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

def testPoseGoToWorkspace(arm=raven_constants.Arm.Right):
    endPose = tfx.pose((0.10111792174636791, 0.018041843424473982, -0.09097057179035087),(-0.7725095288330014, 0.5345941554837077, -0.2807504756969483, 0.19651281683598326))
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)
    ravenArm.start()

    rospy.loginfo('Press enter to go to workspace')
    raw_input()

    ravenArm.executeInterpolatedTrajectory(endPose)

    ravenArm.ravenController.stop()



if __name__ == '__main__':

    testPoseGoToWorkspace()
