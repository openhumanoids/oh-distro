classdef AtlasBehaviorModePublisher

	properties
		lc;
		channel;
  end

  methods
		function obj = AtlasBehaviorModePublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
		function publish(obj, data)
      msg = drc.behavior_command_t();
      msg.utime = data.utime;
      msg.command = data.command;
      obj.lc.publish(obj.channel, msg);
    end
  end

end
