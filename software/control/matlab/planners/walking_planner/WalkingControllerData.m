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

      % do we have to save to file to convert to byte stream?
      msg.qtraj = getByteStreamFromArray(obj.qtraj);
      msg.n_qtraj_bytes = length(msg.qtraj);

      msg.K = getByteStreamFromArray(obj.K);
      msg.n_qtraj_bytes = length(msg.K);

      msg.S = getByteStreamFromArray(obj.S);
      msg.n_S_bytes = length(msg.S);

      msg.s1 = getByteStreamFromArray(obj.s1);
      msg.n_s1_bytes = length(msg.s1);

      msg.s1dot = getByteStreamFromArray(obj.s1dot);
      msg.n_s1dot_bytes = length(msg.s1dot);

      msg.s2 = getByteStreamFromArray(obj.s2);
      msg.n_s2_bytes = length(msg.s2);

      msg.s2dot = getByteStreamFromArray(obj.s2dot);
      msg.n_s2dot_bytes = length(msg.s2dot);

      msg.n_support_times = length(obj.support_times);
      msg.support_times = obj.support_times;

      msg.supports = getByteStreamFromArray(obj.supports);
      msg.n_supports_bytes = length(msg.supports);

      msg.comtraj = getByteStreamFromArray(obj.comtraj);
      msg.n_comtraj_bytes = length(msg.comtraj);

      msg.zmptraj = getByteStreamFromArray(obj.zmptraj);
      msg.n_zmptraj_bytes = length(msg.zmptraj);

      msg.link_constraints = getByteStreamFromArray(obj.link_constraints);
      msg.n_link_constraints_bytes = length(msg.link_constraints);

      msg.mu = obj.mu;
      msg.ignore_terrain = obj.ignore_terrain;
      if isfield(obj,'t_offset')
        msg.t_offset = obj.t_offset;
      else
        msg.t_offset = 0;
      end
    end

  methods (Static = true)
    function obj = from_walking_plan_t(msg_data)

      S = getArrayFromByteStream(typecast(msg_data.S, 'uint8'));

      if isa(S,'Trajectory')
        S = eval(S,0); % S is always constant
      end

      s1 = getArrayFromByteStream(typecast(msg_data.s1, 'uint8'));

      istv = ~(isnumeric(s1) || isa(s1,'ConstantTrajectory'));

      s1dot = getArrayFromByteStream(typecast(msg_data.s1dot, 'uint8'));

      s2 = getArrayFromByteStream(typecast(msg_data.s2, 'uint8'));

      s2dot = getArrayFromByteStream(typecast(msg_data.s2dot, 'uint8'));

      support_times = msg_data.support_times;

      supports = getArrayFromByteStream(typecast(msg_data.supports, 'uint8'));
      if iscell(supports)
        warning('somebody sent me a cell array of supports.  don''t do that anymore!');
        supports = [supports{:}];
      end

      comtraj = getArrayFromByteStream(typecast(msg_data.comtraj, 'uint8'));

      zmptraj = getArrayFromByteStream(typecast(msg_data.zmptraj, 'uint8'));

      link_constraints = getArrayFromByteStream(typecast(msg_data.link_constraints, 'uint8'));

      qtraj = getArrayFromByteStream(typecast(msg_data.qtraj, 'uint8'));

      K = getArrayFromByteStream(typecast(msg_data.K, 'uint8'));

      mu = msg_data.mu;
      ignore_terrain = msg_data.ignore_terrain;
      t_offset = msg_data.t_offset;

      V = struct('S', S, 's1', s1, 's2', s2);
      obj = WalkingControllerData(V, support_times,...
                                     supports, comtraj, mu, t_offset,...
                                     link_constraints, zmptraj, qtraj,...
                                     ignore_terrain,K);
      obj.s1dot = s1dot;
      obj.s2dot = s2dot;
    end
  end
end

