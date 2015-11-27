from director import transformUtils
import numpy

if ikPlanner.pushToMatlab==True:
  print "FAILURE - pushing requests to matlab"
  exit()

qT=numpy.array([0, 0, 0, 0, 0, 0, -6.310489698080346e-05, 0.34103086590766907, 3.8130277971504256e-05, 1.4273228645324707, 5.833456089021638e-05, -0.4845042824745178, -3.8867587136337534e-05])
q0=numpy.array([ 0., 0., 0., 0., 0., 0., 0., 0.78539816, 0.,  1.57079633, 0., -0.78539816, 0.])
goalFrame = transformUtils.frameFromPositionAndRPY([0.36932988056397303, -0.009998017176602909, 0.8891143571732633], [-1.3262913021702864e-12, 89.99999979432002, -89.99963750134272])

constraintSet = ikPlanner.planEndEffectorGoal(q0, 'left', goalFrame, lockBase=True, lockBack=True)
q=numpy.array(constraintSet.runIk()[0])
ret=constraintSet.runIkTraj()

if ((q-qT).__abs__()>1e-3).any():
  print "FAILURE - IK pose incorrect."
  exit()

if ret.plan_info[0]!=0:
  print "FAILURE - Planner failed."
  exit()

# When everything goes all right, report success
with open(os.environ.get('SYSTEMTEST_RESULT_FILE'),'w+') as f:
  f.write('1\n')
exit()
