

# simple example
addSignal('EST_ROBOT_STATE', msg.pose.translation.x)
addSignal('EST_ROBOT_STATE', msg.pose.translation.y)
addSignal('EST_ROBOT_STATE', msg.pose.translation.z)


# function example
def myFunction(msg):
    return msg.utime, msg.trans[1]

addSignal('POSE_BDI', myFunction)
