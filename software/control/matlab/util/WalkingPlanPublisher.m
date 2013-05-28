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
      % assume: data is a struct with fields: htraj, hddtraj, S, s1, supptraj, comtraj, zmptraj, lfoottraj, rfoottraj

      % do we have to save to file to convert to byte stream?
      qtraj = data.qtraj;
      save('tmp_r.mat','qtraj');
      fid = fopen('tmp_r.mat','r');
      msg.qtraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
      fclose(fid);
      msg.n_qtraj_bytes = length(msg.qtraj); 

%       htraj = data.htraj;
%       save('tmp_r.mat','htraj');
%       fid = fopen('tmp_r.mat','r');
%       msg.htraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
%       fclose(fid);
%       msg.n_htraj_bytes = length(msg.htraj); 
% 
%       hddtraj = data.hddtraj;
%       save('tmp_r.mat','hddtraj');
%       fid = fopen('tmp_r.mat','r');
%       msg.hddtraj = fread(fid,inf,'*uint8');
%       fclose(fid);
%       msg.n_hddtraj_bytes = length(msg.hddtraj); 

      S = data.S;
      save('tmp_r.mat','S');
      fid = fopen('tmp_r.mat','r');
      msg.S = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_S_bytes = length(msg.S); 

      s1 = data.s1;
      save('tmp_r.mat','s1');
      fid = fopen('tmp_r.mat','r');
      msg.s1 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s1_bytes = length(msg.s1); 

      s2 = data.s2;
      save('tmp_r.mat','s2');
      fid = fopen('tmp_r.mat','r');
      msg.s2 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s2_bytes = length(msg.s2); 
      
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

      zmptraj = data.zmptraj;
      save('tmp_r.mat','zmptraj');
      fid = fopen('tmp_r.mat','r');
      msg.zmptraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_zmptraj_bytes = length(msg.zmptraj); 

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
      
      qnom = data.qnom;
      save('tmp_r.mat','qnom');
      fid = fopen('tmp_r.mat','r');
      msg.qnom = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_qnom_bytes = length(msg.qnom);
      
      msg.mu = data.mu;

      obj.lc.publish(obj.channel, msg);
		end

	end

end
