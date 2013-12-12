import lcm, os, sys, drc, math, time

"""
Very simple neck controller that spools out neck pitch commands to the atlas driver.

"""

lc = lcm.LCM()

class NeckControl:
    def __init__(self):
        self.delta_max = 0.15;
        self.deadband = 0.05;
        self.cur_neck_pitch = None;
        self.des_neck_pitch = None;

    def cmd_handle(self, channel, data):
        if not self.cur_neck_pitch is None:
            msg = drc.neck_pitch_t.decode(data)
            self.des_neck_pitch = msg.pitch;

    def state_handle(self, channel, data):
        msg = drc.robot_state_t.decode(data)
        neck_idx = [i for i, s in enumerate(msg.joint_name) if 'neck_ay' == s][0]
        self.cur_neck_pitch = msg.joint_position[neck_idx]
        if not self.des_neck_pitch is None:
            command = drc.neck_pitch_t();
            command.utime = msg.utime;
            error = self.des_neck_pitch - self.cur_neck_pitch
            if abs(error) > self.deadband:
                command.pitch = self.cur_neck_pitch + max(-self.delta_max,min(self.delta_max,error));
                lc.publish("COMMANDED_NECK_PITCH", command.encode())
                time.sleep(0.05) # publish at a lower rate

  
def main():
    nc = NeckControl()
    lc.subscribe("DESIRED_NECK_PITCH", nc.cmd_handle)
    lc.subscribe("EST_ROBOT_STATE", nc.state_handle)
    print "Listening for desired neck pitch..."
    while True:
        lc.handle()

if __name__ == '__main__':
    main()
