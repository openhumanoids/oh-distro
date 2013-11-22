classdef AtlasManipParamsListener 
	properties
		monitor;
    channel;
    t_cur = -1;
    x_cur = [];
	end

	methods
		function obj = AtlasManipParamsListener(channel)
      if nargin < 1
        channel = 'ATLAS_STATUS';
      end
      obj.channel = channel;
		  obj.monitor = drake.util.MessageMonitor(drc.atlas_status_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
    end

		function [x,t] = getNextMessage(obj, t_ms)
			data = obj.monitor.getNextMessage(t_ms);
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = drc.atlas_status_t(data);
        x = AtlasManipParamsListener.decode(msg);
			end
    end
    
    function [x,t] = getMessage(obj)
			data = obj.monitor.getMessage();
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = drc.atlas_status_t(data);
        x = AtlasManipParamsListener.decode(msg);
			end
		end
    
  end
  
  methods(Static)
    function data = decode(msg)
      pelvis_servo = msg.manipulate_feedback.internal_desired;  
      data.pelvis_height = pelvis_servo.pelvis_height;
      data.pelvis_yaw = pelvis_servo.pelvis_yaw;
      data.pelvis_pitch = pelvis_servo.pelvis_pitch;
      data.pelvis_roll = pelvis_servo.pelvis_roll;
      data.com_v0 = pelvis_servo.com_v0;
      data.com_v1 = pelvis_servo.com_v1;
    end
  end
end
