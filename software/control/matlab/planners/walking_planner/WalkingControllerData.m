classdef WalkingControllerData
  properties
    S
    s1
    s2
    s1dot
    s2dot
    support_times
    supports
    comtraj
    mu
    t_offset
    link_constraints
    zmptraj
    qtraj
    ignore_terrain
    K
  end

  methods
    function obj = WalkingControllerData(V, support_times,...
                                         supports, comtraj, mu, t_offset,...
                                         link_constraints, zmptraj, qtraj,...
                                         ignore_terrain,K)
      obj.S = V.S;
      obj.s1 = V.s1;
      obj.s2 = V.s2;
      obj.s1dot = fnder(obj.s1,1);
      obj.s2dot = fnder(obj.s2,1);
      obj.support_times = support_times;
      obj.supports = supports;
      obj.comtraj = comtraj;
      obj.mu = mu;
      obj.t_offset = t_offset;
      obj.link_constraints = link_constraints;
      obj.zmptraj = zmptraj;
      obj.qtraj = qtraj;
      obj.ignore_terrain = ignore_terrain;
      obj.K = K;
    end

    function msg = toLCM(obj)
      msg = drc.walking_plan_t();
      
      msg.robot_name = 'atlas';
      msg.utime = 0;
      tmp_fname = ['tmp_r_', num2str(feature('getpid')), '.mat'];

      % do we have to save to file to convert to byte stream?
      qtraj = obj.qtraj;
      save(tmp_fname,'qtraj');
      fid = fopen(tmp_fname,'r');
      msg.qtraj = fread(fid,inf,'*uint8'); % note: this will be stored as an int8 in the lcmtype
      fclose(fid);
      msg.n_qtraj_bytes = length(msg.qtraj); 

      S = obj.S;
      save(tmp_fname,'S');
      fid = fopen(tmp_fname,'r');
      msg.S = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_S_bytes = length(msg.S); 

      s1 = obj.s1;
      save(tmp_fname,'s1');
      fid = fopen(tmp_fname,'r');
      msg.s1 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s1_bytes = length(msg.s1); 

      s1dot = obj.s1dot;
      save(tmp_fname,'s1dot');
      fid = fopen(tmp_fname,'r');
      msg.s1dot = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s1dot_bytes = length(msg.s1dot); 

      s2 = obj.s2;
      save(tmp_fname,'s2');
      fid = fopen(tmp_fname,'r');
      msg.s2 = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s2_bytes = length(msg.s2); 

      s2dot = obj.s2dot;
      save(tmp_fname,'s2dot');
      fid = fopen(tmp_fname,'r');
      msg.s2dot = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_s2dot_bytes = length(msg.s2dot); 

      msg.n_support_times = length(obj.support_times);
      msg.support_times = obj.support_times;
      
      supports = obj.supports;
      save(tmp_fname,'supports');
      fid = fopen(tmp_fname,'r');
      msg.supports = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_supports_bytes = length(msg.supports); 

      comtraj = obj.comtraj;
      save(tmp_fname,'comtraj');
      fid = fopen(tmp_fname,'r');
      msg.comtraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_comtraj_bytes = length(msg.comtraj); 

      zmptraj = obj.zmptraj;
      save(tmp_fname,'zmptraj');
      fid = fopen(tmp_fname,'r');
      msg.zmptraj = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_zmptraj_bytes = length(msg.zmptraj); 

      link_constraints = obj.link_constraints;
      save(tmp_fname, 'link_constraints');
      fid = fopen(tmp_fname, 'r');
      msg.link_constraints = fread(fid,inf,'*uint8');
      fclose(fid);
      msg.n_link_constraints_bytes = length(msg.link_constraints);
      
      msg.mu = obj.mu;
      msg.ignore_terrain = obj.ignore_terrain;
      if isfield(obj,'t_offset')
        msg.t_offset = obj.t_offset;
      else
        msg.t_offset = 0;
      end

      delete(tmp_fname);
    end
  end
end

