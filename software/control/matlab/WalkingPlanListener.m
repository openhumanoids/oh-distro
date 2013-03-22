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
        
        % do we have to save to file to convert a byte stream to a
        % matlab binary?
        
        fid = fopen('htraj_w.mat','w');
        fwrite(fid,typecast(msg.htraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('htraj_w.mat');
        htraj=matdata.htraj;

        fid = fopen('hddtraj_w.mat','w');
        fwrite(fid,typecast(msg.hddtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('hddtraj_w.mat');
        hddtraj=matdata.hddtraj;

        fid = fopen('Straj_w.mat','w');
        fwrite(fid,typecast(msg.Straj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('Straj_w.mat');
        Straj=matdata.Straj;
 
        fid = fopen('qtraj_w.mat','w');
        fwrite(fid,typecast(msg.qtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('qtraj_w.mat');
        qtraj=matdata.qtraj;

        x = struct('qtraj',qtraj,'htraj',htraj,'hddtraj',hddtraj,'Straj',Straj);
			end
    end
	end

end
