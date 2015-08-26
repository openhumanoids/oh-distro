import ipab
import time

if ikPlanner.pushToMatlab==False:
  print "FAILURE - NOT pushing requests to matlab!"
  exit()

## Begin Test ##

# Start the simulator
msg=ipab.scs_api_command_t()
msg.command="simulate"
lcmUtils.publish('SCS_API_CONTROL', msg)
time.sleep(2.0)

footstepsPanel.onNewWalkingGoal()
footstepsPanel.onExecute()

## End Test ##

# When everything goes all right, report success
with open(os.environ.get('SYSTEMTEST_RESULT_FILE'),'w+') as f:
  f.write('1\n')
exit()
