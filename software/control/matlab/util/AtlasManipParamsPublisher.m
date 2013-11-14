classdef AtlasManipParamsPublisher

	properties
		lc;
		channel;
	end

	methods
		function obj = AtlasManipParamsPublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
		function publish(obj, data)
      msg = drc.atlas_behavior_manipulate_params_t();
      msg.use_desired = true;
      msg.use_demo_mode = false;
      msg.desired = drc.atlas_behavior_pelvis_servo_params_t();
      msg.desired.pelvis_height = data.pelvis_height;
      msg.desired.pelvis_yaw = data.pelvis_yaw;
      msg.desired.pelvis_pitch = data.pelvis_pitch;
      msg.desired.pelvis_roll = data.pelvis_roll;
      msg.desired.com_v0 = data.com_v0;
      msg.desired.com_v1 = data.com_v1;
      obj.lc.publish(obj.channel, msg);
    end
  end

end
