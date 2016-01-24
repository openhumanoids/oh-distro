addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
for i in range(4):
    addSignal('VAL_INS_PELVIS_REAR_IMU', msg.utime, msg.quat[i])
for i in range(3):
    addSignal('VAL_INS_PELVIS_REAR_IMU', msg.utime, msg.gyro[i])
    addSignal('VAL_INS_PELVIS_REAR_IMU', msg.utime, msg.accel[i])

addPlot(timeWindow=15, yLimits=[-2.75, 2.75])
for i in range(4):
    addSignal('VAL_INS_LEFT_TORSO_IMU', msg.utime, msg.quat[i])
for i in range(3):
    addSignal('VAL_INS_LEFT_TORSO_IMU', msg.utime, msg.gyro[i])
    addSignal('VAL_INS_LEFT_TORSO_IMU', msg.utime, msg.accel[i])
