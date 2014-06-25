classdef TEMPAtlasCommandPublisher

	properties
		lc;
		channel;
	end

	methods
		function obj = TEMPAtlasCommandPublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
		end

		function publish(obj, utime, seq_id)
      msg = drc.atlas_command_t();
      msg.utime = utime;
      msg.seq_id = seq_id;

      msg.num_joints = 28;
  
      msg.joint_names =              {'back_bkz', 'back_bky', 'back_bkx',... 
         'neck_ay', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', ...
         'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', ...
         'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', ...
         'r_leg_akx', 'l_arm_usy', 'l_arm_shx', 'l_arm_ely', ...
         'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', 'r_arm_usy', ...
         'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'};      
      %= repmat('blah',28,1);
       
      msg.position = zeros(msg.num_joints,1);
      msg.velocity = zeros(msg.num_joints,1);
      msg.effort = zeros(msg.num_joints,1);

      msg.k_q_p = zeros(msg.num_joints,1);
      msg.k_q_i = zeros(msg.num_joints,1);
      msg.k_qd_p = zeros(msg.num_joints,1);
      msg.k_f_p = zeros(msg.num_joints,1);
      msg.ff_qd = zeros(msg.num_joints,1);
      msg.ff_qd_d = zeros(msg.num_joints,1);
      msg.ff_f_d = zeros(msg.num_joints,1);
      msg.ff_const = zeros(msg.num_joints,1);

      msg.k_effort = zeros(msg.num_joints,1);

      msg.desired_controller_period_ms = 0;

      obj.lc.publish(obj.channel, msg);
    end

end

end
