import lcm, os, sys
sys.path.append(os.path.expanduser('~/drc/software/build/lib/python2.7/dist-packages'))
import drc


lc = lcm.LCM()

msg = drc.plan_control_t()
msg.utime = 0
lc.publish("STOP_WALKING", msg.encode())

