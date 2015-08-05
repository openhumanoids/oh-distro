#findLCMTypes(os.path.expanduser('../../build/lib/python2.7/dist-packages/*'))
findLCMTypes(os.path.expanduser('../../build/lib/python2.7/dist-packages/drc/'))

joints = ['l_leg_kny','r_leg_kny']
#addSignals('ATLAS_STATE', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_position, joints, keyLookup=msg.joint_name)


addPlot()
#addSignals('ATLAS_STATE', msg.utime, msg.joint_effort, joints, keyLookup=msg.joint_name)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_effort, joints, keyLookup=msg.joint_name)

addPlot()
#addSignals('ATLAS_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=msg.joint_name)
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_velocity, joints, keyLookup=msg.joint_name)

