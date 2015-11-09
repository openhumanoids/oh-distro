import ipab
import time
import threading
from bot_procman import printf_t
import numpy
listener=lcmUtils.MessageResponseHelper('PMD_PRINTF', printf_t)

footstep_status_ok=False
footstep_pos=[]

def getPrintfListener():
  return 

def footstep_callback(msg):
  global footstep_status_ok
  global footstep_pos
  if msg.COMPLETED and msg.footstep_index==5:
    footstep_pos=numpy.array(list(msg.actual_foot_position_in_world)+list(transformUtils.quaternionToRollPitchYaw(msg.actual_foot_orientation_in_world)))
    footstep_status_ok=True


# Executes the command on the main thread
def run(cmd):
  __ctkConsole.runcode(cmd)

def Tester():
  global listener
  if ikPlanner.pushToMatlab==False:
    print "FAILURE - NOT pushing requests to matlab!"
    exit()
  
  ## Begin Test ##
  
  # Start the simulator
  print "Starting the simulation"
  msg=ipab.scs_api_command_t()
  msg.command="simulate"
  lcmUtils.publish('SCS_API_CONTROL', msg)
  time.sleep(2.0)

  # Wait until the footsteps are produced
  print "Waiting for Matlab to start"

  # Set map mode to horizontal plane
  footstepsPanel.propertiesPanel.children()[1].children()[8].currentIndex=3

  while not om.findObjectByName('step 6'):
    time.sleep(2.0)
    print "Sending footstep plan request"
    run('footstepsPanel.onNewWalkingGoal()')
  t=om.findObjectByName('step 6 frame').transform
  footgoal=numpy.array(t.GetPosition()+t.GetOrientation())
  
  
  # Execute footsteps  
  print "Executing the footstep plan"
  run('footstepsPanel.onExecute()')
  
  # Wait for the motion to finish
  print "Waiting for execution"
  while not footstep_status_ok:
    time.sleep(0.5)

  foot=footstep_pos

  # Check success

  if ((footgoal-foot).__abs__()>1e-3).any():
    print "FAILURE - foot wasn't in the corect position!"
    exit()

  ## End Test ##

  print "Success - the robot arrived at its destination"
  #run("lcmUtils.publish('SCS_API_CONTROL', msgstop)")

  # When everything goes all right, report success
  with open(os.environ.get('SYSTEMTEST_RESULT_FILE'),'w+') as f:
    f.write('1\n')
  time.sleep(0.5)
  run('exit()')

msgstop=ipab.scs_api_command_t()
msgstop.command="stop"

thr = threading.Thread(target=Tester)
thr.setDaemon(True)
lcmUtils.addSubscriber('IHMC_FOOTSTEP_STATUS', messageClass=ipab.footstep_status_t, callback=footstep_callback);
thr.start()




