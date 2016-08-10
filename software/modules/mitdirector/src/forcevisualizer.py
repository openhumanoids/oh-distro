__author__ = 'manuelli'

from director import visualization as vis
from director.debugVis import DebugData
import director.objectmodel as om
from director import lcmUtils
from director import transformUtils
import bot_core
import drc as lcmdrc
import drake as lcmdrake

import numpy as np


class ForceVisualizer:

    def __init__(self, robotSystem, view):

        self.robotStateModel = robotSystem.robotStateModel
        self.robotStateJointController = robotSystem.robotStateJointController
        self.robotSystem = robotSystem
        self.view = view
        self.rFootFtFrameId = self.robotStateModel.model.findFrameID("r_foot_force_torque")
        self.lFootFtFrameId = self.robotStateModel.model.findFrameID("l_foot_force_torque")

        self.leftInContact = 0
        self.rightInContact = 0
        self.footContactEstimateMsg = None
        self.initializeOptions()

        d = DebugData()
        visObj = vis.updatePolyData(d.getPolyData(), self.options['estForceVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)

        visObj = vis.updatePolyData(d.getPolyData(), self.options['pelvisAccelerationVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)

        visObj = vis.updatePolyData(d.getPolyData(), self.options['copVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)

        visObj = vis.updatePolyData(d.getPolyData(), self.options['QPForceVisName'], view=self.view, parent='robot state model')

        visObj = vis.updatePolyData(d.getPolyData(), self.options['desiredCOPVisName'], view=self.view, parent='robot state model')


        self.visObjDict = dict()
        self.visObjDict['bodyMotion'] = vis.updatePolyData(d.getPolyData(), self.options['bodyMotionVisName'], view=self.view, parent='robot state model')
        self.addSubscribers()


        # setup a dict to keep track of the names we will need
        lFootDict = {}
        lFootDict['frameId'] = self.robotStateModel.model.findFrameID("l_foot_force_torque")
        lFootDict['visName'] = 'l_foot_force_est'

        rFootDict = {}
        rFootDict['frameId'] = self.robotStateModel.model.findFrameID("r_foot_force_torque")
        rFootDict['visName'] = 'r_foot_force_est'


        self.nameDict = {}
        self.nameDict['l_foot'] = lFootDict
        self.nameDict['r_foot'] = rFootDict

    def initializeOptions(self):
        self.options = {}
        self.options['speedLimit'] = 20 # speed limit on redrawing
        self.options['forceMagnitudeNormalizer'] = 600
        self.options['forceArrowLength'] = 0.4
        self.options['forceArrowTubeRadius'] = 0.01
        self.options['forceArrowHeadRadius'] = 0.03
        self.options['forceArrowColor'] = [1,0,0]
        self.options['estForceVisName'] = 'meas foot forces'
        self.options['pelvisAccelerationVisName'] = 'desired pelvis acceleration'
        self.options['pelvisMagnitudeNormalizer'] = 2.0
        self.options['pelvisArrowLength'] = 0.8
        self.options['QPForceVisName'] = 'QP foot force'
        self.options['bodyMotionVisName'] = 'Body Motion Data'

        self.options['copVisName'] = 'meas cop'
        self.options['desiredCOPVisName'] = 'desired cop'

        self.options['colors'] = dict()
        self.options['colors']['plan'] = [1,0,0] # red
        self.options['colors']['controller'] = [0,0,1] # blue
        self.options['colors']['measured'] = [0,1,0] # green



    def addSubscribers(self):

        # FORCE_TORQUE subscriber
        # draws measured foot forces, cop etc
        self.forceTorqueSubscriber = lcmUtils.addSubscriber('FORCE_TORQUE', bot_core.six_axis_force_torque_array_t,
                                                            self.onForceTorqueMessage)
        self.forceTorqueSubscriber.setSpeedLimit(self.options['speedLimit'])

        # draw
        self.controllerStateSubscriber = lcmUtils.addSubscriber("CONTROLLER_STATE", lcmdrc.controller_state_t, self.onControllerStateMessage)
        self.controllerStateSubscriber.setSpeedLimit(self.options['speedLimit'])

        # foot contact estimate
        sub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, self.onFootContactEstimateMsg)
        sub.setSpeedLimit(self.options['speedLimit'])

        # desiredCOP
        sub = lcmUtils.addSubscriber('QP_CONTROLLER_INPUT', lcmdrake.lcmt_qp_controller_input, self.extractDesiredCOP)


    # msg is six_axis_force_torque_array_t
    def onForceTorqueMessage(self, msg):

        if (om.findObjectByName(self.options['estForceVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            d = DebugData()

            for idx, footName in enumerate(msg.names):
                self.drawFootForce(footName, msg.sensors[idx], d)

            vis.updatePolyData(d.getPolyData(), name=self.options['estForceVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', self.options['colors']['measured'])


        if (om.findObjectByName(self.options['copVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            copInWorld, d = self.computeCOP(msg)
            vis.updatePolyData(d.getPolyData(), name=self.options['copVisName'], view=self.view,
                           parent='robot state model').setProperty('Color', self.options['colors']['measured'])


    # here msg is six_axis_force_torque_t
    def drawFootForce(self, footName, msg, debugData):
        force = np.array(msg.force)
        forceNorm = np.linalg.norm(force)
        scaledForce = self.options['forceArrowLength']/self.options['forceMagnitudeNormalizer']*force
        torque = np.array(msg.moment)

        visName = self.nameDict[footName]['visName']
        ftFrameId = self.nameDict[footName]['frameId']

        ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)

        forceStartInWorld = ftFrameToWorld.TransformPoint((0,0,0))
        forceEndInWorld = np.array(ftFrameToWorld.TransformPoint(scaledForce))

        debugData.addArrow(forceStartInWorld, forceEndInWorld, tubeRadius=self.options['forceArrowTubeRadius'],
                           headRadius=self.options['forceArrowHeadRadius'], color=self.options['colors']['measured'])


    def drawQPContactWrench(self, msg):
        # draw the contact wrenches
        d = DebugData()
        copData = dict()
        totalForce = np.zeros(3)
        for contact_output in msg.contact_output:
            footName = contact_output.body_name
            ftFrameId = 0
            ftFrameToWorld = 0
            if (footName == 'leftFoot' or footName == 'l_foot'):
                footName = "left"
                ftFrameId = self.nameDict['l_foot']['frameId']
                ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)
            elif (footName == 'rightFoot' or footName == 'r_foot'):
                footName = "right"
                ftFrameId = self.nameDict['r_foot']['frameId']
                ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)

            #arrowStart = np.array((msg.contact_ref_points[i][0], msg.contact_ref_points[i][1], msg.contact_ref_points[i][2]))
            arrowStart = ftFrameToWorld.TransformPoint((0,0,0))
            force = np.array((contact_output.wrench[3], contact_output.wrench[4], contact_output.wrench[5]))
            arrowEnd = arrowStart + self.options['forceArrowLength']/self.options['forceMagnitudeNormalizer']*force

            d.addArrow(arrowStart, arrowEnd, tubeRadius=self.options['forceArrowTubeRadius'],
                           headRadius=self.options['forceArrowHeadRadius'], color=self.options['colors']['controller'])

            # compute cop
            cop = np.zeros(3)
            cop[0] = -contact_output.wrench[1] / contact_output.wrench[5] + contact_output.ref_point[0]
            cop[1] = contact_output.wrench[0] / contact_output.wrench[5] + contact_output.ref_point[1]
            cop[2] = contact_output.ref_point[2]

            totalForce += np.array((contact_output.wrench[3],contact_output.wrench[4],contact_output.wrench[5]))

            data = dict()
            data['cop'] = cop
            data['fz'] = contact_output.wrench[5]
            copData[footName] = data

        cop = np.zeros(3)
        totalForceMag = 0
        for key, val in copData.iteritems():
            totalForceMag += val['fz']

        for key, val in copData.iteritems():
            cop += val['cop']*val['fz']/totalForceMag


        bottomFootZVal = -0.09
        if(np.abs(totalForce[2]) > 0.01):
            cop += bottomFootZVal/totalForce[2]*totalForce

        d.addSphere(cop, radius=0.015)
        vis.updatePolyData(d.getPolyData(), name=self.options['QPForceVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', self.options['colors']['controller'])

    # draw the acceleration of the pelvis that the QP thinks is happening
    def onControllerStateMessage(self, msg):

        if (om.findObjectByName(self.options['pelvisAccelerationVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):

            pelvisJointNames = ['base_x', 'base_y', 'base_z']
            pelvisAcceleration = np.zeros(3)

            for idx, name in enumerate(pelvisJointNames):
                stateIdx = msg.joint_name.index(name)
                pelvisAcceleration[idx] = msg.qdd[stateIdx]


            pelvisFrame = self.robotStateModel.getLinkFrame('pelvis')

            arrowStart = np.array(pelvisFrame.TransformPoint((0,0,0)))
            arrowEnd = arrowStart + self.options['pelvisArrowLength']*np.linalg.norm(pelvisAcceleration)/self.options['pelvisMagnitudeNormalizer']*pelvisAcceleration

            debugData = DebugData()
            debugData.addArrow(arrowStart, arrowEnd, tubeRadius=self.options['forceArrowTubeRadius'],
                               headRadius=self.options['forceArrowHeadRadius'])

            vis.updatePolyData(debugData.getPolyData(), name=self.options['pelvisAccelerationVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', self.options['colors']['controller'])

        if (om.findObjectByName(self.options['QPForceVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            self.drawQPContactWrench(msg)


        if (om.findObjectByName(self.options['bodyMotionVisName']).getProperty('Visible')):
            self.drawBodyMotionData(msg.desired_body_motions)


        # print "got controller state message"
        # print "pelvisAcceleration ", pelvisAcceleration
        # print "arrowStart ", arrowStart
        # print "arrowEnd ", arrowEnd


    def computeCOP(self, msg):
        d = DebugData()
        copDataList = []
        for idx, footName in enumerate(msg.names):
            copDataList.append(self.computeSingleFootCOP(footName, msg.sensors[idx], d))

        copInWorld = (copDataList[0]['cop']*copDataList[0]['fz'] + copDataList[1]['cop']*copDataList[1]['fz'])/(
            copDataList[0]['fz'] + copDataList[1]['fz'])

        d.addSphere(copInWorld, radius=0.015)


        return copInWorld, d


    def computeSingleFootCOP(self, footName, msg, d):

        force = np.array(msg.force)
        # hack for dealing with very small fz.
        if np.linalg.norm(force) < 0.001:
            force[2] = 0.001


        x = -msg.moment[1]/force[2]
        y = msg.moment[0]/force[2]
        z_foot = 0.035
        alpha = z_foot/force[2]

        cop = np.array((x,y,0))
        cop = cop + alpha*force

        ftFrameId = self.nameDict[footName]['frameId']
        ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)

        copInWorld = np.array(ftFrameToWorld.TransformPoint(cop))

        d.addSphere(copInWorld, radius=0.01)

        copData = {'cop': copInWorld, 'fz': np.linalg.norm(force)}
        return copData



    # record which foot is in contact
    def onFootContactEstimateMsg(self, msg):
        self.footContactEstimateMsg = msg

    # get current average foot height
    def getAverageFootHeight(self):
        if self.footContactEstimateMsg is None:
            return 0
        footNames = ['l_foot', 'r_foot']
        footContact = np.array([self.footContactEstimateMsg.left_contact, self.footContactEstimateMsg.right_contact])
        avgFootHeight = 0

        for idx, name in enumerate(footNames):
            ftFrameId = self.nameDict[name]['frameId']
            ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)
            soleHeight = ftFrameToWorld.GetPosition()[2]
            avgFootHeight += footContact[idx]*soleHeight

        if np.sum(footContact) > 0.1:
            avgFootHeight = avgFootHeight/np.sum(footContact)
        else:
            avgFootHeight = 0

        return avgFootHeight

    # computes the desired COP location coming from the qp_input message
    def extractDesiredCOP(self, qpInputMsg):
        if (om.findObjectByName(self.options['desiredCOPVisName']).getProperty('Visible')):
            y0 = qpInputMsg.zmp_data.y0
            desiredCOP = np.zeros(3)
            desiredCOP[0] = y0[0][0]
            desiredCOP[1] = y0[1][0]
            desiredCOP[2] = self.getAverageFootHeight()


            d = DebugData()
            d.addSphere(desiredCOP, radius=0.015)

            vis.updatePolyData(d.getPolyData(), name=self.options['desiredCOPVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', self.options['colors']['plan'])



    # takes in a qp_desired_body_motion_t message
    def drawBodyMotionData(self, desired_body_motions):
        # first remove all existing frames
        childFrames = self.visObjDict['bodyMotion'].children()
        for frame in childFrames:
            om.removeFromObjectModel(frame)

        # add new frames for tracked bodies
        for bodyMotionData in desired_body_motions:
            bodyName = bodyMotionData.body_name
            frameName = bodyName + ' body motion'
            position = bodyMotionData.body_q_d[0:3]
            rpy = bodyMotionData.body_q_d[3:6]
            frame = transformUtils.frameFromPositionAndRPY(position, rpy)
            vis.showFrame(frame, frameName, view=self.view, parent=self.options['bodyMotionVisName'], scale=0.2)




























