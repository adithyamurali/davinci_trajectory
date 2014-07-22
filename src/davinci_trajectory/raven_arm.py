#!/usr/bin/env python

import roslib; roslib.load_manifest('raven_2_trajectory')
import numpy as np
import os
import math

import rospy

import code

import tfx

import IPython

# import openravepy as rave


# rename so no conflict with raven_2_msgs.msg.Constants
from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController

class RavenArm:
    """
    Class for controlling the end effectors of the Raven
    """

    def __init__ (self, armName, closedGraspValue=0.,defaultPoseSpeed=.01):
        self.armName = armName

        if armName == raven_constants.Arm.Left:
            self.toolFrame = raven_constants.Frames.LeftTool
        else:
            self.toolFrame = raven_constants.Frames.RightTool
            
        # self.commandFrame = raven_constants.Frames.Link0
        self.commandFrame = "two_remote_center_link"
        
        self.ravenController = RavenController(self.armName, closedGraspValue=closedGraspValue, defaultPoseSpeed=defaultPoseSpeed)

 
    #############################
    # start, pause, and stop ####
    #############################
        
    def start(self):
        """
        Starts the raven arm (releases the e-brake)
        (does nothing if raven arm is already running)
        """
        print 'starting raven arm ', self.armName
        return self.ravenController.start()

    def pause(self):
        """
        Pauses by clearing the stages
        """
        self.ravenController.clearStages()

    def isPaused(self):
        """
        True if no stages
        """
        return len(self.ravenController.stages) == 0
        
    def stop(self):
        return self.ravenController.stop()

    #######################
    # trajectory commands #
    #######################

    def executePoseTrajectory(self, poseTraj, block=True, duration=None, speed=None, ignoreOrientation=False):
        """
        poseTraj is a tuple/list of poses

        Can set either duration or speed (not both)

        duration is either the time of the whole trajectory
        or a list of the duration of each segment of the trajectory

        speed is the joint speed
        """
        if duration != None:
            if type(duration) != tuple:
                duration = [duration/len(poseTraj) for _ in range(len(poseTraj))]
            
            if len(duration) != len(poseTraj):
                return

        prevPose = None
        for pose in poseTraj:
            self.goToGripperPose(tfx.pose(pose), startPose=prevPose, block=False, duration=duration, speed=speed, ignoreOrientation=ignoreOrientation)
            prevPose = pose

        if block:
            return self.blockUntilPaused()
        return True

    def executeStampedPoseTrajectory(self, stampedPoses, block=True):
        """
        stampedPoses is a list of tfx poses with time stamps
        """

        prevTime = rospy.Time.now()
        prevPose = None

        for stampedPose in stampedPoses:
            duration = (stampedPose.stamp - prevTime).to_sec()

            pose = tfx.pose(stampedPose.position, stampedPose.orientation)
            self.goToGripperPose(pose, startPose=prevPose, block=False, duration=duration)

            prevTime = stampedPose.stamp
            prevPose = pose

        if block:
            return self.blockUntilPaused()
        return True
    
    def executeDeltaPoseTrajectory(self, deltaPoses, startPose=None, block=True, speed=None, ignoreOrientation=False):
        """
        Each deltaPose in deltaPoses is with respect to the startPose
        
        Assuming deltaPoses and startPose in same frame (0_link)
        """
        if startPose is None:
            startPose = self.getGripperPose()
            if startPose is None:
                return
        
        endPoses = [raven_util.endPose(startPose, deltaPose, self.commandFrame) for deltaPose in deltaPoses]
        #IPython.embed()
        print
        print 'START', startPose
        print 'END', endPoses[-1]
        print
        '''IPython.embed()
        '''
        return self.executePoseTrajectory(endPoses, block=block, speed=speed, ignoreOrientation=ignoreOrientation)
            

    def executeStampedDeltaPoseTrajectory(self, stampedDeltaPoses, startPose=None, block=True):
        """
        stampedDeltaPoses is a list of tfx poses with time stamps

        each stampedDeltaPose is endPose - startPose, where
        each endPose is different but the startPose is the same

        This function is intended to be used where each
        endPose is from the vision system and the one
        startPose is the gripper position according to vision
        """
        durations = []
        prevTime = rospy.Time.now()
        for stampedDeltaPose in stampedDeltaPoses:
            durations.append(stampedDeltaPose.stamp - prevTime)
            prevTime = stampedDeltaPose.stamp

        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return

        startPose = raven_util.convertToFrame(tfx.pose(startPose), self.commandFrame)

        endPoses = []
        for deltaPose in stampedDeltaPoses:
            endPoses.append(raven_util.endPose(startPose, deltaPose, self.commandFrame))

        prevPose = None
        for duration, endPose in zip(duration, endPoses):
            self.goToGripperPose(endPose, startPose=prevPose, block=False, duration=duration)
            prevPose = endPose

        if block:
            return self.blockUntilPaused()
        return True

    def executeJointTrajectory(self, jointTraj, block=True, duration=None, speed=None):
        """
        jointTraj is a tuple/list of joints dictionaries

        joints is a dictionary of {jointType : jointPos}

        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians

        Can set either duration or speed (not both)

        duration is either the time of the whole trajectory
        or a list of the duration of each segment of the trajectory

        speed is a scalar factor (compared to default joint speed)
        """
        if duration != None:
            if type(duration) != tuple:
                duration = [duration/len(jointTraj) for _ in range(len(jointTraj))]
            
            if len(duration) != len(jointTraj):
                return

        prevJoints = None
        for joints in jointTraj:
            joints = dict(joints)
            self.goToJoints(joints, startJoints=prevJoints, block=False, duration=duration, speed=speed)
            prevJoints = joints

        if block:
            self.blockUntilPaused()
        return True

    def executeStampedJointTrajectory(self, stampedJoints, block=True):
        """
        stampedJoints is a tuple of ((stamp, joints), ....)

        joints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        
        prevTime = rospy.Time.now()
        prevJoints = None

        for stamp, joints in stampedJoints:
            duration = (stamp - prevTime).to_sec()
            joints = dict(joints) # so we don't clobber, just in case
            
            self.goToJoints(joints, startJoints=prevJoints, block=False, duration=duration)

            prevTime = stamp
            prevJoints = joints

        if block:
            return self.blockUntilPaused()
        return True

    #####################
    # command methods   #
    #####################

    def goToGripperPose(self, endPose, startPose=None, block=True, duration=None, speed=None, ignoreOrientation=False):
        """        
        Move to endPose

        If startPose is not specified, default to current pose according to tf
        (w.r.t. Link0)
        """
        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return

        startPose = raven_util.convertToFrame(tfx.pose(startPose), self.commandFrame)
        endPose = raven_util.convertToFrame(tfx.pose(endPose), self.commandFrame)


        if ignoreOrientation:
            endPose.orientation = startPose.orientation

        self.ravenController.goToPose(endPose, start=startPose, duration=duration, speed=speed)

        if block:
            return self.blockUntilPaused()
        return True

    def goToGripperPoseDelta(self, deltaPose, startPose=None, block=True, duration=None, speed=None, ignoreOrientation=False):
        """
        Move by deltaPose

        If startPose is not specified, default to current pose according to tf
        (w.r.t. Link0)
        """
        if startPose == None:
            startPose = self.getGripperPose()
            if startPose == None:
                return

        endPose = raven_util.endPose(startPose, deltaPose, self.commandFrame)

        self.goToGripperPose(endPose, startPose=startPose, block=block, duration=duration, speed=speed, ignoreOrientation=ignoreOrientation)


    def goToJoints(self, desJoints, startJoints=None, block=True, duration=None, speed=None):
        """
        desJoints is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        
        speed is a factor gain compared to default speeds
        """
        
        self.ravenController.goToJoints(desJoints, startJoints=startJoints, duration=duration, speed=speed)

        if block:
            return self.blockUntilPaused()
        return True
            
    def closeGripper(self,duration=2.5, block=True):
        self.setGripper(0.,duration=duration,block=block)

                
    def openGripper(self,duration=2.5, block=True):
        self.setGripper(1., duration=duration, block=block)

    def setGripper(self, grasp,duration=2.5, block=True):
        self.ravenController.clearStages()
        self.ravenController.setGripper(grasp,duration=duration)
        
        if block:
            rospy.sleep(duration)

    def setGripperPositionDaVinci(self, angle):
        self.ravenController.setGripperPositionDaVinci(angle)

    #######################
    # state info methods  #
    #######################

    def getGripperPose(self,frame=raven_constants.Frames.Link0):
        """
        Returns gripper pose w.r.t. frame

        geometry_msgs.msg.Pose
        """
        ### This return statement was added so as not to bother with frames
        return self.ravenController.currentPose


        pose = tfx.pose([0,0,0],frame=self.toolFrame)
        return tfx.convertToFrame(pose, self.commandFrame, pose.frame, wait=10)
        
        gripperPose = self.ravenController.currentPose

        if gripperPose != None:
            gripperPose = raven_util.convertToFrame(tfx.pose(gripperPose), frame)

        return gripperPose

    def getCurrentJoints(self):
        """
        Returns is dictionary of {jointType : jointPos}
        
        jointType is from raven_2_msgs.msg.Constants
        jointPos is position in radians
        """
        return self.ravenController.getCurrentJoints()

    def executeInterpolatedTrajectory(self, endPose, num_steps = 1):
        rospy.sleep(1)

        # rospy.loginfo('Press enter to execute Trajectory')
        # raw_input()

        n_steps = num_steps
        weight = float(1)/n_steps
        trajectory = []

        startPose = self.ravenController.currentPose

        for i in range(n_steps):
            trajectory.append(startPose.interpolate(endPose, weight * (i + 1)))

        for pose in trajectory:
            self.goToGripperPose(pose)

        # rospy.loginfo('Press enter to exit')
        # raw_input()

    #################
    # other methods #
    #################
    
    @property
    def name(self):
        return self.armName
    
    def blockUntilPaused(self, timeoutTime=999999):
        timeout = raven_util.Timeout(timeoutTime)
        timeout.start()

        while not self.isPaused():
            if rospy.is_shutdown() or timeout.hasTimedOut():
                return False
            rospy.sleep(.05)

        return True


def testOpenCloseGripper(close=True,arm=raven_constants.Arm.Right):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)

    ravenArm.start()
    
    rospy.loginfo('Press enter to set the gripper')
    raw_input()
    
    rospy.loginfo('Setting the gripper')
    if close:
        ravenArm.closeGripper()
    else:
        ravenArm.openGripper()

    rospy.loginfo('Press enter to exit')
    raw_input()

def testMoveToHome(arm=raven_constants.Arm.Right):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    imageDetector = ARImageDetector()
    rospy.sleep(4)

    ravenArm.start()

    rospy.loginfo('Press enter to go to home')
    raw_input()

    #desPose = imageDetector.getReceptaclePose()
    desPose = imageDetector.getHomePose()
    ravenArm.goToGripperPose(desPose)
    
    rospy.loginfo('Press enter to exit')
    raw_input()
    
def testGoToJoints(arm=raven_constants.Arm.Right):
    rospy.init_node('rave_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(0,90,0)
    endPose = tfx.pose([-.073, -.014, -.124], angle,frame=raven_constants.Frames.Link0)


    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose using joint commands')
    raw_input()

    desJoints = ravenPlanner.getJointsFromPose(endPose)
    currJoints = ravenArm.getCurrentJoints()
    
    """
    grasp = ravenPlanner.currentGrasp
    yaw = desJoints[Constants.JOINT_TYPE_YAW]
    finger1 = yaw - grasp/2
    finger2 = -(yaw + grasp/2)
    desJoints[Constants.JOINT_TYPE_GRASP_FINGER1] = finger1
    desJoints[Constants.JOINT_TYPE_GRASP_FINGER2] = finger2
    del desJoints[Constants.JOINT_TYPE_YAW]
    """

    rospy.loginfo('Found joints')
    for jointType, jointPos in desJoints.items():
        print("desired: jointType = {0}, jointPos = {1}".format(jointType,jointPos))
        print("current: jointType = {0}, jointPos = {1}".format(jointType,currJoints[jointType]))

    #code.interact(local=locals())

    rospy.loginfo('Press enter to move')
    raw_input()
    
    ravenArm.goToJoints(desJoints)

    #code.interact(local=locals())

    rospy.loginfo('Press enter to exit')
    raw_input()

def testExecuteTrajopt(arm=raven_constants.Arm.Right):
    rospy.init_node('rave_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(0,90,0)
    endPose = tfx.pose([-.073, -.014, -.15], angle,frame=raven_constants.Frames.Link0)


    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose using joint commands')
    raw_input()

    desJoints = ravenPlanner.getJointsFromPose(endPose)
    currJoints = ravenArm.getCurrentJoints()

    endJointTraj = ravenPlanner.getTrajectoryFromPose(endPose)

    rospy.loginfo('Found joints')
    for jointType, jointPos in desJoints.items():
        print("desired: jointType = {0}, jointPos = {1}".format(jointType,jointPos))
        print("current: jointType = {0}, jointPos = {1}".format(jointType,currJoints[jointType]))


    rospy.loginfo('Press enter to move')
    raw_input()
    
    ravenArm.executeJointTrajectory(endJointTraj)
    #ravenArm.goToJoints(endJointTraj[-1])

    code.interact(local=locals())

    rospy.loginfo('Press enter to exit')
    raw_input()   

def testGoToPose(arm=raven_constants.Arm.Right):
    rospy.init_node('raven_arm_node',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(20,70,0)
    endPose = tfx.pose([-.136, -.017, -.075], angle,frame=raven_constants.Frames.Link0)

    ravenArm.start()

    rospy.loginfo('Press enter to go to endPose')
    raw_input()

    ravenArm.goToGripperPose(endPose)

    rospy.loginfo('Press enter to exit')
    raw_input()
    

def testTrajopt(arm=raven_constants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    angle = tfx.tb_angles(90,90,0)
    endPose = tfx.pose([-.133, -.015, -.1], angle,frame=raven_constants.Frames.Link0)
    # z -.072

    startJoints = ravenArm.getCurrentJoints()

    endJoints = ravenPlanner.getJointsFromPose(endPose)

    rospy.loginfo('desired and start joint positions')
    for jointType, jointPos in endJoints.items():
        print("desired: jointType = {0}, jointPos = {1}".format(jointType,(180.0/math.pi)*jointPos))
        print("current: jointType = {0}, jointPos = {1}".format(jointType,(180.0/math.pi)*startJoints[jointType]))

    rospy.loginfo('Press enter to call trajopt')
    raw_input()

    jointTraj = ravenPlanner.getTrajectoryFromPose(startJoints, endPose)

    for jointDict in jointTraj:
        print(jointDict)

    """
    for trajIndex in range(len(jointTraj)):
        jointWaypoint = jointTraj[trajIndex]
        print(list((180.0/math.pi)*jointWaypoint))

    endJointPositions = jointTraj[-1]
    endTrajJoints = dict(zip(startJoints.keys()[:-1], list(endJointPositions)))     
    """

    ravenPlanner.env.SetViewer('qtcoin')
    #ravenPlanner.updateOpenraveJoints(jointTraj[-1])

    rospy.loginfo('Successful use of trajopt')
    code.interact(local=locals())

def testExecuteJointTrajectory(arm=raven_constants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(4)


    if arm == raven_constants.Arm.Right:
        toolframe = raven_constants.Frames.RightTool
    else:
        toolframe = raven_constants.Frames.LeftTool

    currPose = tfx.pose([0,0,0], frame=toolframe)
    tf_tool_to_link0 = tfx.lookupTransform(raven_constants.Frames.Link0, currPose.frame, wait=5)
    currPose = tf_tool_to_link0 * currPose

    angle = tfx.tb_angles(-90,90,0)
    endPosition = currPose.position
    endPosition.x -= .05
    endPose = tfx.pose(endPosition, angle,frame=raven_constants.Frames.Link0)


    startJoints = ravenArm.getCurrentJoints()

    ####### TEMP
    # so adding box will work, don't normally need to do though
    ravenPlanner.updateOpenraveJoints(startJoints)
    # box = rave.RaveCreateKinBody(ravenPlanner.env,'')
    box.SetName('testbox')
    box.InitFromBoxes(np.array([[0,-.025,0,0.1,0.01,0.01]]),True)
    #code.interact(local=locals())
    ee = ravenPlanner.manip.GetEndEffectorTransform()
    ee[:3,:3] = np.identity(3)
    box.SetTransform(ee)
    ravenPlanner.env.Add(box,True)
    #ravenPlanner.env.SetViewer('qtcoin')
    rospy.loginfo('Box created, press enter')
    ravenFile = os.path.dirname(__file__) + '/../../../models/myRavenEnv.uri'
    ravenPlanner.env.Save(ravenFile)
    #code.interact(local=locals())
    return
    
    ###########

    #ravenPlanner.env.SetViewer('qtcoin')

    rospy.loginfo('Press enter to call trajopt')
    raw_input()
    rospy.sleep(1)

    jointTraj = ravenPlanner.getTrajectoryFromPose(endPose, reqType=Request.Type.Pose)
    
    if jointTraj == None:
        return
    
    rospy.loginfo('Press enter to move')
    raw_input()

    #ravenArm.start()
    
    #ravenArm.executeJointTrajectory(jointTraj)
    
    
    #ravenPlanner.updateOpenraveJoints(jointTraj[0])

    endJoints = ravenPlanner.getJointsFromPose(endPose)
    ravenPlanner.updateOpenraveJoints(endJoints)
    endJointsEE = ravenPlanner.manip.GetEndEffectorTransform()

    ravenPlanner.updateOpenraveJoints(jointTraj[-1])
    jointTrajEE = ravenPlanner.manip.GetEndEffectorTransform()

    #raven_util.plot_transform(ravenPlanner.env, endJointsEE)
    #raven_util.plot_transform(ravenPlanner.env, jointTrajEE)

    code.interact(local=locals())

    rospy.loginfo('Press enter to exit')
    raw_input()

def testOpenraveJoints(arm=raven_constants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenPlanner(arm)
    rospy.sleep(2)

    startJoints = ravenArm.getCurrentJoints()

    ravenPlanner.updateOpenraveJoints(startJoints)

    ravenPlanner.env.SetViewer('qtcoin')

    code.interact(local=locals())
    
def testExecuteJointTrajectoryBSP(arm=raven_constants.Arm.Right):
    rospy.init_node('test_trajopt',anonymous=True)
    ravenArm = RavenArm(arm)
    ravenPlanner = RavenBSP(arm)
    rospy.sleep(4)


    if arm == raven_constants.Arm.Right:
        toolframe = raven_constants.Frames.RightTool
    else:
        toolframe = raven_constants.Frames.LeftTool

    currPose = tfx.pose([0,0,0], frame=toolframe)
    tf_tool_to_link0 = tfx.lookupTransform(raven_constants.Frames.Link0, currPose.frame, wait=5)
    currPose = tf_tool_to_link0 * currPose

    angle = tfx.tb_angles(-90,90,0)
    endPosition = currPose.position
    endPosition.y += .05
    endPose = tfx.pose(endPosition, angle,frame=raven_constants.Frames.Link0)


    rospy.loginfo('Press enter to call bsp')
    raw_input()

    jointTraj = ravenPlanner.getTrajectoryFromPose(endPose)
    
    if jointTraj == None:
        return
    
    rospy.loginfo('Press enter to move')
    raw_input()

    ravenArm.start()
    rospy.sleep(1)
    
    ravenArm.executeJointTrajectory(jointTraj)
    
    
    rospy.loginfo('Press enter to exit')
    raw_input()

def testGripperMove(arm=raven_constants.Arm.Left):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)

    ravenArm.start()

    rospy.loginfo('Press enter to start')
    raw_input()

    startPose = ravenArm.ravenController.currentPose
    delta = tfx.pose([0.01, 0.01, 0.01])
    endPose = raven_util.endPose(startPose, delta)    

    print "Start Pose:"
    print startPose
    print "End Pose:"
    print endPose
    ravenArm.ravenController.goToPose(endPose)

    rospy.loginfo('Press enter to stop')
    raw_input()

    ravenArm.ravenController.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()

def testGripperLinearInterpolator(arm=raven_constants.Arm.Right):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)
    ravenArm.start()

    rospy.loginfo('Press enter to start')
    raw_input()

    n_steps = 10
    weight = float(1)/n_steps
    trajectory = []
    trajectory1 = []

    startPose = ravenArm.ravenController.currentPose
    # delta = tfx.pose([0.00, -0.05, 0.05])
    delta = tfx.pose([0.00, 0.00, 0.00])
    endPose = raven_util.endPose(startPose, delta)
    # ravenArm.setGripperPositionDaVinci(0.756851)
    for i in range(n_steps):
        trajectory.append(startPose.interpolate(endPose, weight * (i + 1)))

    # ravenArm.setGripperPositionDaVinci(0.01)
    print "Start Pose:", startPose
    for pose in trajectory:
        print repr(pose)
        # ravenArm.goToGripperPose(pose)
        # rospy.sleep(1)
    print "End pose:", endPose

    # startPose1 = ravenArm.getGripperPose()
    # delta1 = tfx.pose([-0.01, -0.01, -0.01])
    # endPose1 = raven_util.endPose(startPose1, delta1)

    # for i in range(n_steps):
    #     trajectory1.append(startPose1.interpolate(endPose1, weight * (i + 1)))

    # print "Start Pose:", startPose1
    # for pose in trajectory1:
    #     print pose
    #     ravenArm.goToGripperPose(pose)
    #     # rospy.sleep(1)
    # print "End pose:", endPose1

    ravenArm.ravenController.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()

def testGripperPoseExecuteEndPose(arm=raven_constants.Arm.Right):
    endPose = tfx.pose((0.10111792174636791, 0.018041843424473982, -0.09097057179035087),(-0.7725095288330014, 0.5345941554837077, -0.2807504756969483, 0.19651281683598326))
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)
    ravenArm.start()

    rospy.loginfo('Press enter to go to workspace')
    raw_input()

    # ravenArm.setGripperPositionDaVinci(0.768574)
    ravenArm.executeInterpolatedTrajectory(endPose)

    ravenArm.ravenController.stop()

def testGripperSetOpenAngle(arm=raven_constants.Arm.Right):
    rospy.init_node('raven_commander',anonymous=True)
    ravenArm = RavenArm(arm)
    rospy.sleep(1)
    ravenArm.start()

    rospy.loginfo('Press enter to start')
    raw_input()

    # ravenArm.executeInterpolatedTrajectory(endPose, 10)

    ravenArm.ravenController.stop()

    rospy.loginfo('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    #testOpenCloseGripper(close=True)
    #testMoveToHome()
    #testGoToJoints()
    #testExecuteTrajopt()
    #testGoToPose()
    #testTrajopt()
    #testExecuteJointTrajectory()
    #testOpenraveJoints()
    #testExecuteJointTrajectoryBSP()
    # testGripperMove()
    # testGripperLinearInterpolator()
    # testGripperPoseExecuteEndPose()
    # testGripperSetOpenAngle()
    print 'hello'
