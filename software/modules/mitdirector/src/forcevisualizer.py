__author__ = 'manuelli'

from director import visualization as vis
from director.debugVis import DebugData
import director.objectmodel as om
from director import lcmUtils
from director import transformUtils
import bot_core
import drc as lcmdrc

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
        self.initializeOptions()

        d = DebugData()
        visObj = vis.updatePolyData(d.getPolyData(), self.options['estForceVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)

        visObj = vis.updatePolyData(d.getPolyData(), self.options['pelvisAccelerationVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)

        visObj = vis.updatePolyData(d.getPolyData(), self.options['copVisName'], view=self.view, parent='robot state model')
        # visObj.setProperty('Visible', False)
        
        visObj = vis.updatePolyData(d.getPolyData(), self.options['QPForceVisName'], view=self.view, parent='robot state model')

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
        self.options['forceMagnitudeNormalizer'] = 600
        self.options['forceArrowLength'] = 0.4
        self.options['forceArrowTubeRadius'] = 0.01
        self.options['forceArrowHeadRadius'] = 0.03
        self.options['forceArrowColor'] = [1,0,0]
        self.options['estForceVisName'] = 'est foot forces'
        self.options['pelvisAccelerationVisName'] = 'desired pelvis acceleration'
        self.options['pelvisMagnitudeNormalizer'] = 2.0
        self.options['pelvisArrowLength'] = 0.8
        self.options['QPForceVisName'] = 'QP foot force'

        self.options['copVisName'] = 'cop'



    def addSubscribers(self):

        # FORCE_TORQUE subscriber
        self.forceTorqueSubscriber = lcmUtils.addSubscriber('FORCE_TORQUE', bot_core.six_axis_force_torque_array_t,
                                                            self.onForceTorqueMessage)
        self.forceTorqueSubscriber.setSpeedLimit(60);

        self.controllerStateSubscriber = lcmUtils.addSubscriber("CONTROLLER_STATE", lcmdrc.controller_state_t, self.onControllerStateMessage)
        self.controllerStateSubscriber.setSpeedLimit(60)


    # msg is six_axis_force_torque_array_t
    def onForceTorqueMessage(self, msg):

        if (om.findObjectByName(self.options['estForceVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            d = DebugData()

            for idx, footName in enumerate(msg.names):
                self.drawFootForce(footName, msg.sensors[idx], d)

            vis.updatePolyData(d.getPolyData(), name=self.options['estForceVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', [1,0,0])


        if om.findObjectByName(self.options['copVisName']):
            self.computeCOP(msg)


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
                           headRadius=self.options['forceArrowHeadRadius'], color=self.options['forceArrowColor'])


    def drawQPContactWrench(self, msg):
        # draw the contact wrenches
        d = DebugData()
        for i, wrench in enumerate(msg.contact_wrenches):
            
            footName = ""
            ftFrameId = 0
            ftFrameToWorld = 0
            if (i == 0):
                ftFrameId = self.nameDict['r_foot']['frameId']
                ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)
            else:
                ftFrameId = self.nameDict['l_foot']['frameId']
                ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)


            #arrowStart = np.array((msg.contact_ref_points[i][0], msg.contact_ref_points[i][1], msg.contact_ref_points[i][2]))
            arrowStart = ftFrameToWorld.TransformPoint((0,0,0))
            force = np.array((wrench[3], wrench[4], wrench[5]))
            arrowEnd = arrowStart + self.options['forceArrowLength']/self.options['forceMagnitudeNormalizer']*force
            
            d.addArrow(arrowStart, arrowEnd, tubeRadius=self.options['forceArrowTubeRadius'],
                           headRadius=self.options['forceArrowHeadRadius'], color=self.options['forceArrowColor'])

        vis.updatePolyData(d.getPolyData(), name=self.options['QPForceVisName'], view=self.view,
                               parent='robot state model').setProperty('Color', [0,0,1])

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
                               parent='robot state model').setProperty('Color', [0,1,0])

        if (om.findObjectByName(self.options['QPForceVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            self.drawQPContactWrench(msg)
        
            
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

        d.addSphere(copInWorld, radius=0.02)
        vis.updatePolyData(d.getPolyData(), name='cop', view=self.view,
                           parent='robot state model').setProperty('Color', [0,1,0])


    def computeSingleFootCOP(self, footName, msg, d):
        x = -msg.moment[1]/msg.force[2]
        y = msg.moment[0]/msg.force[2]
        z_foot = 0.035
        alpha = z_foot/msg.force[2]
        force = np.array(msg.force)
        cop = np.array((x,y,0))
        cop = cop + alpha*force

        ftFrameId = self.nameDict[footName]['frameId']
        ftFrameToWorld = self.robotStateModel.getFrameToWorld(ftFrameId)

        copInWorld = np.array(ftFrameToWorld.TransformPoint(cop))

        d.addSphere(copInWorld, radius=0.01)

        copData = {'cop': copInWorld, 'fz': np.linalg.norm(force)}
        return copData





























