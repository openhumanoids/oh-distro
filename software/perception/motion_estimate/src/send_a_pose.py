import lcm
from bot_core.pose_t import pose_t

msg = pose_t()
msg.utime = 0
msg.pos = (3, 2.1, 0)
msg.orientation = (1, 0, 0, 0)

lc = lcm.LCM()
lc.publish("POSE_HEAD", msg.encode())
