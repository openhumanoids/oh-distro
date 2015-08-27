import ipab
import time
import threading

def Tester():
  if ikPlanner.pushToMatlab==False:
    print "FAILURE - NOT pushing requests to matlab!"
    exit()
  
  ## Begin Test ##
  
  # Start the simulator
  msg=ipab.scs_api_command_t()
  msg.command="simulate"
  lcmUtils.publish('SCS_API_CONTROL', msg)
  time.sleep(90.0)
  time.sleep(2.0)
  
  footstepsPanel.onNewWalkingGoal()
  time.sleep(10.0)
  footstepsPanel.onExecute()
  time.sleep(30.0)
  
  ## End Test ##

  # When everything goes all right, report success
  with open(os.environ.get('SYSTEMTEST_RESULT_FILE'),'w+') as f:
    f.write('1\n')
  exit()

thr = threading.Thread(target=Tester)
thr.setDaemon(True)
thr.start()


