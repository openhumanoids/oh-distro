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

		function publish(obj, data, utime)
      msg = drc.walking_plan_t();
      
 			msg.robot_name = 'atlas';
      msg.utime = utime;
      % assume: data = struct('qtraj',qtraj,'htraj',htraj,'hddtraj',hddtraj,'Straj',Straj);

      % do we have to save to file to convert to byte stream?
      htraj = data.htraj;
      save('htraj_tmp.mat','htraj');
      fid = fopen('htraj_tmp.mat','r');
      msg.htraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_htraj_bytes = length(msg.htraj); 

      hddtraj = data.hddtraj;
      save('hddtraj_tmp.mat','hddtraj');
      fid = fopen('hddtraj_tmp.mat','r');
      msg.hddtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_hddtraj_bytes = length(msg.hddtraj); 

      Straj = data.Straj;
      save('Straj_tmp.mat','Straj');
      fid = fopen('Straj_tmp.mat','r');
      msg.Straj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_Straj_bytes = length(msg.Straj); 

      qtraj = data.qtraj;
      save('qtraj_tmp.mat','qtraj');
      fid = fopen('qtraj_tmp.mat','r');
      msg.qtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_qtraj_bytes = length(msg.qtraj); 

      obj.lc.publish(obj.channel, msg);
		end

	end

end
