classdef WalkingPlanListener 
	properties
		monitor;
    channel;
    t_cur = -1;
    x_cur = [];
	end

	methods
		function obj = WalkingPlanListener(channel)
      obj.channel = channel;
		  obj.monitor = drake.util.MessageMonitor(drc.walking_plan_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
    end

		function [x,t] = getNextMessage(obj, t_ms)
			data = obj.monitor.getNextMessage(t_ms);
			if isempty(data)
				x = [];
        t = -1;
      else
        msg = drc.walking_plan_t(data);
        t = obj.monitor.getLastTimestamp();
        
        msg.qtraj
        
        % do we have to save to file to convert a byte stream to a
        % matlab binary?
        
        fid = fopen('htraj_tmp.mat','w');
        fwrite(fid,msg.htraj,'uint8');
        fclose(fid);
        matdata = load('htraj_tmp.mat');
        matdata.htraj;

        fid = fopen('hddtraj_tmp.mat','w');
        fwrite(fid,msg.hddtraj,'uint8');
        fclose(fid);
        matdata = load('hddtraj_tmp.mat');
        matdata.hddtraj;

        fid = fopen('Straj_tmp.mat','w');
        fwrite(fid,msg.Straj,'uint8');
        fclose(fid);
        matdata = load('Straj_tmp.mat');
        matdata.Straj;
 
        fid = fopen('qtraj_tmp.mat','w');
        fwrite(fid,msg.qtraj,'uint8');
        fclose(fid);
        matdata = load('qtraj_tmp.mat');
        matdata.qtraj;

        x = struct('qtraj',qtraj,'htraj',htraj,'hddtraj',hddtraj,'Straj',Straj);
			end
    end
	end

end
