import lcm
import drc.map_pointcloud_request_t
import time

def sendRequest():
	msg = drc.map_pointcloud_request_t()
	msg.utime = long(time.time()*1000)
	msg.resolution = 0
	msg.spindle_angle_start = 0.1
	msg.num_revolutions = 0.1
	msg.min_range = 0.3
	msg.max_range = 5
	msg.view_id = 100
	theLcm = lcm.LCM()
	theLcm.publish('MAP_POINTCLOUD_REQUEST', msg.encode())
	
def repeatRequest():
	while True:
		sendRequest()
		time.sleep(2.0)
