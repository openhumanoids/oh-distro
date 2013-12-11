import lcm, os, sys
sys.path.append(os.path.expanduser('~/drc/software/build/lib/python2.7/dist-packages'))
import drc

lc = lcm.LCM()

msg = drc.atlas_behavior_command_t()
msg.utime = 0
msg.command = "ladder_reach_up:RIGHT"
lc.publish("LADDER_REACH", msg.encode())

