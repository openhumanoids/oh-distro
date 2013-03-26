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
        
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.htraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        htraj=matdata.htraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.hddtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        hddtraj=matdata.hddtraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.Straj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        Straj=matdata.Straj;
 
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.qtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        qtraj=matdata.qtraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.supptraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        supptraj=matdata.supptraj;

        x = struct('qtraj',qtraj,'htraj',htraj,'hddtraj',hddtraj,'Straj',Straj,'supptraj',supptraj);
			end
    end
	end

end
