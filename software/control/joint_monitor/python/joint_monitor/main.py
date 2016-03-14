from numpy import *
import lcm, os, sys, drc, time
import bot_core as lcmbotcore

"""
Simple process that publishes joint positions and forces scaled to [-1,1] using 
joint limits and maximum joint torques.
"""

lc = lcm.LCM()

class JointMonitor:
    def __init__(self):
        self.joint_names = ['back_bkz', 'back_bky','back_bkx','l_arm_usy','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_leg_hpz', \
                        'l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx','l_arm_mwx', \
                        'r_arm_usy','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_uwy','r_leg_hpz', \
                        'r_leg_hpx','r_leg_hpy','r_leg_kny','r_leg_aky','r_leg_akx','r_arm_mwx','neck_ay']

        jlmin = [-0.6632,-0.6107,-0.6981,-1.5708,-1.5708,0.0,0.0,0.0,-0.1745,-0.5236,-1.7207,0.0,-1.0,-0.8, \
            -1.1781,-1.5708,-1.5708,0.0,-2.3562,0.0,-1.2217,-0.5236,-1.7207,0.0,-1.0,-0.8,-1.1781,-0.6021]
        jlmax = [0.6632,0.49,0.6981,0.7854,1.5708,3.1416,2.3562,3.1416,1.2217,0.5236,0.5248,2.3857,0.7, \
            0.8,1.1781,0.7854,1.5708,3.1416,0.0,3.1416,0.1745,0.5236,0.5248,2.3857,0.7,0.8,1.1781,1.1449]

        # all except back.  currently made up numbers
        jtorquemax = [114.0000, 206.8430, 200.0000, 75, 207, 55, 55, 55, 110, 180, 260, 220, 700, 90, 55, 75, 207, 55, 55, 55, 110, 180, 260, 220, 700, 90, 55, 5]
        jtorquemin = [-114.0000, -206.8430, -200.0000, -75, -207, -55, -55, -55, -110, -180, -260, -220, -700, -90, -55, -75, -207, -55, -55, -55, -110, -180, -260, -220, -700, -90, -55,-5]

#        jtorquemax = [100,100,50,50,50,1000,1000,1000,1000,1000,1000,50,100,100,50,50,50,1000,1000,1000,1000,1000,1000,50,100];    
#        jtorquemin = [-100,-100,-50,-50,-50,-1000,-1000,-1000,-1000,-1000,-1000,-50,-100,-100,-50,-50,-50,-1000,-1000,-1000,-1000,-1000,-1000,-50,-100];    

        bkx_pos = [-0.837758,-0.77998161,-0.72220522,-0.66442883,-0.60665244,-0.54887605,-0.49109963,-0.4333232, \
            -0.37554678,-0.31777036,-0.25999394,-0.20221752,-0.1444411,-0.08666468,-0.02888827,0.02888814,0.08666456, \
            0.14444098, 0.2022174, 0.25999382, 0.31777024, 0.37554666, 0.43332309, 0.49109951, 0.54887593, 0.60665232, \
            0.66442871, 0.7222051, 0.77998149, 0.83775789]
        bkx_torque_max = dict(zip(bkx_pos,[222.9618,234.8207,246.1703,256.9747,267.1978,276.8028,285.7514,294.0050,301.5239,308.2675,314.1944, \
            319.2626,323.4291,326.6507,328.8837,330.0842,330.2087,329.2144,327.0592,323.7033,319.1089,313.2416,306.0710, \
            297.5722,287.7265,276.5231,263.9600,250.0460,234.8013,218.2595]))
        bkx_torque_min = dict(zip(bkx_pos,[-302.5634,-318.6563,-334.0579,-348.7196,-362.5927,-375.6267,-387.7702,-398.9705,-409.1738,-418.3250, \
            -426.3679,-433.2456,-438.8996,-443.2714,-446.3016,-447.9307,-448.0996,-446.7503,-443.8258,-439.2717,-433.0370, \
            -425.0750,-415.3443,-403.8113,-390.4505,-375.2472,-358.1989,-339.3173,-318.6299,-296.1824]))

        bkx_pos = [-0.837758,-0.77998161,-0.72220522,-0.66442883,-0.60665244,-0.54887605,-0.49109963,-0.4333232, \
            -0.37554678,-0.31777036,-0.25999394,-0.20221752,-0.1444411,-0.08666468,-0.02888827,0.02888814,0.08666456, \
            0.14444098, 0.2022174, 0.25999382, 0.31777024, 0.37554666, 0.43332309, 0.49109951, 0.54887593, 0.60665232, \
            0.66442871, 0.7222051, 0.77998149, 0.83775789]
        bkx_torque_max = dict(zip(bkx_pos,[222.9618,234.8207,246.1703,256.9747,267.1978,276.8028,285.7514,294.0050,301.5239,308.2675,314.1944, \
            319.2626,323.4291,326.6507,328.8837,330.0842,330.2087,329.2144,327.0592,323.7033,319.1089,313.2416,306.0710, \
            297.5722,287.7265,276.5231,263.9600,250.0460,234.8013,218.2595]))
        bkx_torque_min = dict(zip(bkx_pos,[-302.5634,-318.6563,-334.0579,-348.7196,-362.5927,-375.6267,-387.7702,-398.9705,-409.1738,-418.3250, \
            -426.3679,-433.2456,-438.8996,-443.2714,-446.3016,-447.9307,-448.0996,-446.7503,-443.8258,-439.2717,-433.0370, \
            -425.0750,-415.3443,-403.8113,-390.4505,-375.2472,-358.1989,-339.3173,-318.6299,-296.1824]))

        bkz_torque_max = bkx_torque_max;
        bky_torque_max = bkx_torque_max;

        bkz_torque_min = bkx_torque_min;
        bky_torque_min = bkx_torque_min;

        self.joint_min = dict(zip(self.joint_names,jlmin))
        self.joint_max = dict(zip(self.joint_names,jlmax))
#        self.joint_torque_max = dict(zip(self.joint_names[0:3],[bkz_torque_max, bky_torque_max, bkx_torque_max]))
#        self.joint_torque_min = dict(zip(self.joint_names[0:3],[bkz_torque_min, bky_torque_min, bkx_torque_min]))
        self.joint_torque_max = dict()
        self.joint_torque_min = dict()
        for i in range(0,len(jtorquemax)):
            self.joint_torque_max[self.joint_names[i]] = dict([(0, jtorquemax[i])])
            self.joint_torque_min[self.joint_names[i]] = dict([(0, jtorquemin[i])])
   
    def get_torque_lims(self,joint_name,joint_pos):
        torque_max_dict = self.joint_torque_max.get(joint_name)
        torque_min_dict = self.joint_torque_min.get(joint_name)
        if torque_max_dict is not None:
            torque_max = torque_max_dict.get(joint_pos, torque_max_dict[min(torque_max_dict.keys(), key=lambda k: abs(k-joint_pos))])
        else:
            torque_max = 1
        if torque_min_dict is not None:
            torque_min = torque_min_dict.get(joint_pos, torque_min_dict[min(torque_min_dict.keys(), key=lambda k: abs(k-joint_pos))])
        else:
            torque_min = -1
        return [torque_min, torque_max]
  
    def get_joint_lims(self,joint_name):
        jmin = self.joint_min.get(joint_name)
        jmax = self.joint_max.get(joint_name)
        if jmin is None:
          jmin = -1
        
        if jmax is None:
          jmax = 1
        return [jmin, jmax]

    def cmd_handle(self, channel, data):
        msg = lcmbotcore.atlas_command_t.decode(data)

    def state_handle(self, channel, data):
        msg = drc.robot_state_t.decode(data)
        msg_out = drc.robot_state_t()
        msg_out.utime = 1e6*time.time()
        msg_out.joint_name = msg.joint_name
        msg_out.num_joints = msg.num_joints
        msg_out.joint_effort = [0]*msg.num_joints
        msg_out.joint_position = [0]*msg.num_joints
        msg_out.joint_velocity = [0]*msg.num_joints
        msg_out.force_torque = drc.force_torque_t()
        msg_out.pose = drc.position_3d_t()
        msg_out.pose.translation = drc.vector_3d_t()
        msg_out.pose.rotation = drc.quaternion_t()
        msg_out.twist = drc.twist_t()
        msg_out.twist.linear_velocity = drc.vector_3d_t()
        msg_out.twist.angular_velocity = drc.vector_3d_t()
        for i in range(0,msg.num_joints):
            torque_lims = self.get_torque_lims(msg.joint_name[i],msg.joint_position[i])
            position_lims = self.get_joint_lims(msg.joint_name[i])
#            print(position_lims)
            msg_out.joint_effort[i] = (min(100,max(-100,-1 + 2*(msg.joint_effort[i] - torque_lims[0])/(torque_lims[1] - torque_lims[0]))))
            msg_out.joint_position[i] = (min(100,max(-100,-1 + 2*(msg.joint_position[i] - position_lims[0])/(position_lims[1] - position_lims[0]))))
#            print("lims: " + str(torque_lims) + " torque: " + str(msg.joint_effort[i]) + " scaled: " + str(msg_out.joint_effort[i]))
#            print("jlims: " + str(position_lims) + " position: " + str(msg.joint_position[i]) + " scaled: " + str(msg_out.joint_position[i]))
        lc.publish("SCALED_ROBOT_STATE", msg_out.encode())
        time.sleep(.01)

def main():
    jmon = JointMonitor()
    lc.subscribe("ATLAS_COMMAND", jmon.cmd_handle)
    lc.subscribe("EST_ROBOT_STATE", jmon.state_handle)
    print "Joint monitor running."
    while True:
        lc.handle()

if __name__ == '__main__':
    main()
