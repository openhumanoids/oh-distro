classdef WalkingPlanPublisher

	properties
		lc;
		channel;
	end

	methods
		function obj = WalkingPlanPublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
		end

		function publish(obj, utime, data)
      msg = data.toLCM();
      obj.lc.publish(obj.channel, msg);
	end

end

end
