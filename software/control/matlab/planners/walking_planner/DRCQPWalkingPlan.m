classdef DRCQPWalkingPlan < QPWalkingPlan
  methods
    function obj = DRCQPWalkingPlan(x0, support_times, supports, ...
           link_constraints, zmptraj, D_ls, V, c, comtraj, mu, ignore_terrain,...
           t_offset, gain_set, biped)
      obj = obj@QPWalkingPlan(biped);
      % Feed straight in
      obj.x0 = x0;
      obj.support_times = support_times;
      obj.supports = supports;
      obj.link_constraints = link_constraints;
      obj.zmptraj = zmptraj;
      obj.D_ls = D_ls;
      obj.V = V;
      obj.c = c;
      obj.comtraj = comtraj;
      obj.mu = mu;
      obj.t_offset = t_offset;
      obj.ignore_terrain = ignore_terrain;
      obj.gain_set = gain_set;
      obj.robot = biped;

      % generate some handy other things
      obj.duration = obj.support_times(end)-obj.support_times(1)-0.001;
      obj.zmp_final = obj.zmptraj.eval(obj.zmptraj.tspan(end));
      if isa(obj.V.S, 'ConstantTrajectory')
        obj.V.S = fasteval(obj.V.S, 0);
      end
    end

    function msg = toLCM(obj)
      msg = drc.walking_plan_t();

      msg.robot_name = 'atlas';
      msg.utime = 0;
      msg.xtraj = mxSerialize(obj.x0);
      msg.n_xtraj_bytes = length(msg.xtraj);

      msg.S = mxSerialize(obj.V.S);
      msg.n_S_bytes = length(msg.S);

      msg.s1 = mxSerialize(obj.V.s1);
      msg.n_s1_bytes = length(msg.s1);

      % msg.s1dot = mxSerialize(obj.s1dot);
      % msg.n_s1dot_bytes = length(msg.s1dot);

      msg.s2 = mxSerialize(obj.V.s2);
      msg.n_s2_bytes = length(msg.s2);

      % msg.s2dot = mxSerialize(obj.s2dot);
      % msg.n_s2dot_bytes = length(msg.s2dot);

      msg.n_support_times = length(obj.support_times);
      msg.support_times = obj.support_times;

      msg.supports = mxSerialize(obj.supports);
      msg.n_supports_bytes = length(msg.supports);

      msg.comtraj = mxSerialize(obj.comtraj);
      msg.n_comtraj_bytes = length(msg.comtraj);

      msg.zmptraj = mxSerialize(obj.zmptraj);
      msg.n_zmptraj_bytes = length(msg.zmptraj);

      msg.D_ls = mxSerialize(obj.D_ls);
      msg.n_D_ls_bytes = length(msg.D_ls);

      msg.link_constraints = mxSerialize(obj.link_constraints);
      msg.n_link_constraints_bytes = length(msg.link_constraints);

      msg.mu = mean(obj.mu);
      msg.ignore_terrain = obj.ignore_terrain;

      msg.gain_set = obj.gain_set;

      if isfield(obj,'t_offset')
        msg.t_offset = obj.t_offset;
      else
        msg.t_offset = 0;
      end
    end
  end

  methods (Static = true)

    function obj = from_drake_walking_data(data, biped)
      % ugly order-dependent unpacking
      obj = DRCQPWalkingPlan(data.x0, data.support_times, data.supports, ...
           data.link_constraints, data.zmptraj, data.D_ls, data.V, data.c, data.comtraj, ...
           data.mu, data.ignore_terrain,...
           data.t_offset, data.gain_set, biped);
    end

    function obj = from_walking_plan_t(msg_data, biped)

      S = mxDeserialize(msg_data.S);

      if isa(S,'Trajectory')
        S = fasteval(S,0); % S is always constant
      end

      s1 = mxDeserialize(msg_data.s1);
%       s1dot = mxDeserialize(msg_data.s1dot);
      s2 = mxDeserialize(msg_data.s2, 'uint8');
%       s2dot = mxDeserialize(msg_data.s2dot);
      supports = mxDeserialize(msg_data.supports);
      if iscell(supports)
        supports = [supports{:}];
      end
      comtraj = mxDeserialize(msg_data.comtraj);
      zmptraj = mxDeserialize(msg_data.zmptraj);
      D_ls = mxDeserialize(msg_data.D_ls);
      link_constraints = mxDeserialize(msg_data.link_constraints);
      x0 = mxDeserialize(msg_data.xtraj);
      gain_set = char(msg_data.gain_set);

      V = struct('S', S, 's1', s1, 's2', s2);

      obj = DRCQPWalkingPlan(x0,...
       msg_data.support_times,...
       supports,...
       link_constraints,...
       zmptraj,...
       D_ls,...
       V,...
       [],...
       comtraj,...
       msg_data.mu,...
       msg_data.ignore_terrain,...
       msg_data.t_offset, ...
       gain_set, ...
       biped);
    end
  end
end

