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
      % assume: data is a struct with fields: htraj, hddtraj, S, s1, 
      %    support_times, supports, comtraj, zmptraj, lfoottraj, rfoottraj
      tmp_fname = ['tmp_r_', num2str(feature('getpid')), '.mat'];

      % do we have to save to file to convert to byte stream?
      qtraj = data.qtraj;
      save(tmp_fname,'qtraj');
      fid = fopen(tmp_fname,'r');
      msg.qtraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
      fclose(fid);
      msg.n_qtraj_bytes = length(msg.qtraj); 

%       htraj = data.htraj;
%       save(tmp_fname,'htraj');
%       fid = fopen(tmp_fname,'r');
%       msg.htraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
%       fclose(fid);
%       msg.n_htraj_bytes = length(msg.htraj); 
% 
%       hddtraj = data.hddtraj;
%       save(tmp_fname,'hddtraj');
%       fid = fopen(tmp_fname,'r');
%       msg.hddtraj = fread(fid,inf,'*uint8');
%       fclose(fid);
%       msg.n_hddtraj_bytes = length(msg.hddtraj); 

      S = data.S;
      save(tmp_fname,'S');
      fid = fopen(tmp_fname,'r');
      msg.S = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_S_bytes = length(msg.S); 

      s1 = data.s1;
      save(tmp_fname,'s1');
      fid = fopen(tmp_fname,'r');
      msg.s1 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s1_bytes = length(msg.s1); 

      s2 = data.s2;
      save(tmp_fname,'s2');
      fid = fopen(tmp_fname,'r');
      msg.s2 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s2_bytes = length(msg.s2); 
      
      msg.n_support_times = length(data.support_times);
      msg.support_times = data.support_times;
      
      supports = data.supports;
      save(tmp_fname,'supports');
      fid = fopen(tmp_fname,'r');
      msg.supports = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_supports_bytes = length(msg.supports); 

      comtraj = data.comtraj;
      save(tmp_fname,'comtraj');
      fid = fopen(tmp_fname,'r');
      msg.comtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_comtraj_bytes = length(msg.comtraj); 

      zmptraj = data.zmptraj;
      save(tmp_fname,'zmptraj');
      fid = fopen(tmp_fname,'r');
      msg.zmptraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_zmptraj_bytes = length(msg.zmptraj); 

      link_constraints = data.link_constraints;
      save(tmp_fname, 'link_constraints');
      fid = fopen(tmp_fname, 'r');
      msg.link_constraints = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_link_constraints_bytes = length(msg.link_constraints);

      % lfoottraj = data.lfoottraj;
      % save(tmp_fname,'lfoottraj');
      % fid = fopen(tmp_fname,'r');
      % msg.lfoottraj = fread(fid,inf,'*uint8');
      % fclose(fid);
      % msg.n_lfoottraj_bytes = length(msg.lfoottraj); 
      
      % rfoottraj = data.rfoottraj;
      % save(tmp_fname,'rfoottraj');
      % fid = fopen(tmp_fname,'r');
      % msg.rfoottraj = fread(fid,inf,'*uint8');
      % fclose(fid);
      % msg.n_rfoottraj_bytes = length(msg.rfoottraj); 
      
      qnom = data.qnom;
      save(tmp_fname,'qnom');
      fid = fopen(tmp_fname,'r');
      msg.qnom = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_qnom_bytes = length(msg.qnom);
      
      msg.mu = data.mu;

      obj.lc.publish(obj.channel, msg);
		end

	end

end
