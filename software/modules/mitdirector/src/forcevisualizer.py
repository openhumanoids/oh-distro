__author__ = 'manuelli'

from director import visualization as vis
from director.debugVis import DebugData
import director.objectmodel as om
from director import lcmUtils
import bot_core

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

    def addSubscribers(self):

        # FORCE_TORQUE subscriber
        self.forceTorqueSubscriber = lcmUtils.addSubscriber('FORCE_TORQUE', bot_core.six_axis_force_torque_array_t,
                                                            self.onForceTorqueMessage)
        self.forceTorqueSubscriber.setSpeedLimit(60);


    # msg is six_axis_force_torque_array_t
    def onForceTorqueMessage(self, msg):

        if not (om.findObjectByName(self.options['estForceVisName']).getProperty('Visible') and self.robotStateJointController.lastRobotStateMessage):
            return

        d = DebugData()

        for idx, footName in enumerate(msg.names):
            self.drawFootForce(footName, msg.sensors[idx], d)

        vis.updatePolyData(d.getPolyData(), name=self.options['estForceVisName'], view=self.view,
                           parent='robot state model').setProperty('Color', [1,0,0])


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
















