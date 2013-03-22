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
      msg = drc.walking_plan_t();
      
 			msg.robot_name = 'atlas';
      msg.utime = utime;
      % assume: data = struct('qtraj',qtraj,'htraj',htraj,'hddtraj',hddtraj,'Straj',Straj);

      % do we have to save to file to convert to byte stream?
      htraj = data.htraj;
      save('htraj_r.mat','htraj');
      fid = fopen('htraj_r.mat','r');
      msg.htraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
      fclose(fid);
      msg.n_htraj_bytes = length(msg.htraj); 

      hddtraj = data.hddtraj;
      save('hddtraj_r.mat','hddtraj');
      fid = fopen('hddtraj_r.mat','r');
      msg.hddtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_hddtraj_bytes = length(msg.hddtraj); 

      Straj = data.Straj;
      save('Straj_r.mat','Straj');
      fid = fopen('Straj_r.mat','r');
      msg.Straj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_Straj_bytes = length(msg.Straj); 

      qtraj = data.qtraj;
      save('qtraj_r.mat','qtraj');
      fid = fopen('qtraj_r.mat','r');
      msg.qtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_qtraj_bytes = length(msg.qtraj); 

      obj.lc.publish(obj.channel, msg);
		end

	end

end
