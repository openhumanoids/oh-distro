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
        t = obj.monitor.getLastTimestamp();
        msg = drc.walking_plan_t(data);
        x = decode(msg);
			end
    end
  end
  
  methods(Static)
    function walking_data = decode(msg)
        % do we have to save to file to convert a byte stream to a
        % matlab binary?
        
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.qtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        qtraj=matdata.qtraj;
        
%         fid = fopen('tmp_w.mat','w');
%         fwrite(fid,typecast(msg.htraj,'uint8'),'uint8');
%         fclose(fid);
%         matdata = load('tmp_w.mat');
%         htraj=matdata.htraj;
% 
%         fid = fopen('tmp_w.mat','w');
%         fwrite(fid,typecast(msg.hddtraj,'uint8'),'uint8');
%         fclose(fid);
%         matdata = load('tmp_w.mat');
%         hddtraj=matdata.hddtraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.S,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        S=matdata.S;
 
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.s1,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        s1=matdata.s1;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.s2,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        s2=matdata.s2;

        support_times=msg.support_times;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.supports,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        supports=matdata.supports;
        
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.comtraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        comtraj=matdata.comtraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.zmptraj,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        zmptraj=matdata.zmptraj;

        % fid = fopen('tmp_w.mat','w');
        % fwrite(fid,typecast(msg.lfoottraj,'uint8'),'uint8');
        % fclose(fid);
        % matdata = load('tmp_w.mat');
        % lfoottraj=matdata.lfoottraj;

        % fid = fopen('tmp_w.mat','w');
        % fwrite(fid,typecast(msg.rfoottraj,'uint8'),'uint8');
        % fclose(fid);
        % matdata = load('tmp_w.mat');
        % foottraj=matdata.rfoottraj;

        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.link_constraints,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        link_constraints=matdata.link_constraints;
        
        fid = fopen('tmp_w.mat','w');
        fwrite(fid,typecast(msg.qnom,'uint8'),'uint8');
        fclose(fid);
        matdata = load('tmp_w.mat');
        qnom = matdata.qnom;
        
        mu = msg.mu;
        
        walking_data = struct('qtraj',qtraj,'mu',mu,...
          'comtraj',comtraj,'zmptraj',zmptraj,'link_constraints',link_constraints,...
          'S',S,'s1',s1,'s2',s2,...
          'support_times',support_times,'supports',supports,'qnom',qnom);

    end
  end

end
