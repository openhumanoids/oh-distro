import lcm
import time
from bot_lcmgl import lcmgl

class Behavior:
    WALKING = 0
    CRAWLING = 1
    BDI_WALKING = 2
    BDI_STEPPING = 3

lc = lcm.LCM()
gl = lcmgl('BDI_Step_Translator', lc)

def now_utime():
    return int(time.time() * 1000000)
