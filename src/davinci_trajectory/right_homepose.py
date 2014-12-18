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

def testPoseHomePose(arm=raven_constants.Arm.Right):
    homePose = tfx.pose([0.08110923304266986, 0.019082072847487756, -0.07564648655601992],(-0.7296703092609553, 0.5879730580371108, -0.28914218075416975, 0.19561626239715652))
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm, False)
    rospy.sleep(1)
    ravenArm.start()


    rospy.loginfo('Press enter to go to Home Pose')
    raw_input()

    print ravenArm.getGripperPose()
    ravenArm.setGripperPositionDaVinci(1)
    ravenArm.goToGripperPose(homePose)

    ravenArm.ravenController.stop()

if __name__ == '__main__':

    testPoseHomePose()
