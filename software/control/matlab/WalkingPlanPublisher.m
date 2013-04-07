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
      % assume: data is a struct with fields: htraj, hddtraj, Straj, supptraj, comtraj, lfoottraj, rfoottraj

      % do we have to save to file to convert to byte stream?
      htraj = data.htraj;
      save('tmp_r.mat','htraj');
      fid = fopen('tmp_r.mat','r');
      msg.htraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
      fclose(fid);
      msg.n_htraj_bytes = length(msg.htraj); 

      hddtraj = data.hddtraj;
      save('tmp_r.mat','hddtraj');
      fid = fopen('tmp_r.mat','r');
      msg.hddtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_hddtraj_bytes = length(msg.hddtraj); 

      Straj = data.Straj;
      save('tmp_r.mat','Straj');
      fid = fopen('tmp_r.mat','r');
      msg.Straj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_Straj_bytes = length(msg.Straj); 

      supptraj = data.supptraj;
      save('tmp_r.mat','supptraj');
      fid = fopen('tmp_r.mat','r');
      msg.supptraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_supptraj_bytes = length(msg.supptraj); 

      comtraj = data.comtraj;
      save('tmp_r.mat','comtraj');
      fid = fopen('tmp_r.mat','r');
      msg.comtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_comtraj_bytes = length(msg.comtraj); 

      lfoottraj = data.lfoottraj;
      save('tmp_r.mat','lfoottraj');
      fid = fopen('tmp_r.mat','r');
      msg.lfoottraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_lfoottraj_bytes = length(msg.lfoottraj); 
      
      rfoottraj = data.rfoottraj;
      save('tmp_r.mat','rfoottraj');
      fid = fopen('tmp_r.mat','r');
      msg.rfoottraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_rfoottraj_bytes = length(msg.rfoottraj); 
      
      obj.lc.publish(obj.channel, msg);
		end

	end

end
