Currently these hands are support:
- robotiq
- no hands
- pointer (see details for modifying director_config.json file)
- valkyrie
- schunk

- to use no hands leave an empty handCombinations block in the json file i.e. "handCombinations": []

1. Using the pointer requires these frames to be put in a json:
            handLinkName = l_hand
            handUrdf = pointer_hand_left.urdf
            handJointName = left_hook_hand_joint
            handRootLink = left_base_link
            robotMountLink = l_hand_force_torque
            palmLink = left_pointer_tip


2. To support iRobot again this code would be required required:
(I think its because the root link of the iRobot hand is inside the mesh)

            self.handLinkName = '%s_hand' % self.side[0]
            self.handUrdf = 'irobot_hand_%s.urdf' % self.side
            self.handJointName = '%s_irobot_hand_joint' % self.side

            if self.side == 'left':
                baseToPalm = [0.0, -0.20516, -0.015, 0.0, 0.0, 0.0]
            else:
                baseToPalm = [0.0, -0.20516, 0.015, 0.0, 0.0, math.radians(180)]


            handRootLink = '%s_base_link' % self.side
            robotMountLink = '%s_hand' % self.side[0]
            palmLink = '%s_hand_face' % self.side[0]
            robotMountToPalm = toFrame(baseToPalm)

            self.loadHandModel()

            baseToHandRoot = self.getLinkToLinkTransform(self.handModel, 'plane::xy::base', handRootLink)
            robotMountToHandRoot = self.getLinkToLinkTransform(robotModel, robotMountLink, handRootLink)

            t = vtk.vtkTransform()
            t.PostMultiply()
            t.Concatenate(baseToHandRoot)
            t.Concatenate(robotMountToHandRoot.GetLinearInverse())
            t.Concatenate(robotMountToPalm)
            self.modelToPalm = t

            self.handLinkToPalm = robotMountToPalm
            self.palmToHandLink = self.handLinkToPalm.GetLinearInverse()




