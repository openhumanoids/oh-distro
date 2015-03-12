classdef DRCQPWalkingPlan < QPWalkingPlan
  methods (Static)
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

      % msg.s2 = mxSerialize(obj.V.s2);
      % msg.n_s2_bytes = length(msg.s2);

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

      msg.D_ls = obj.D_ls;

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

    function obj = from_walking_plan_t(msg_data, biped)

      S = mxDeserialize(msg_data.S);

      if isa(S,'Trajectory')
        S = fasteval(S,0); % S is always constant
      end

      s1 = mxDeserialize(msg_data.s1);
%       s1dot = mxDeserialize(msg_data.s1dot);
      % s2 = mxDeserialize(msg_data.s2, 'uint8');
%       s2dot = mxDeserialize(msg_data.s2dot);
      supports = mxDeserialize(msg_data.supports);
      if iscell(supports)
        supports = [supports{:}];
      end
      x0 = mxDeserialize(msg_data.xtraj);

      obj = QPWalkingPlan(biped);
      obj.x0 = x0;
      obj.support_times = double(msg_data.support_times);
      obj.supports = supports;
      obj.link_constraints = mxDeserialize(msg_data.link_constraints);
      obj.zmptraj = mxDeserialize(msg_data.zmptraj);
      obj.zmp_final = fasteval(obj.zmptraj, obj.zmptraj.tspan(end));
      obj.D_ls = msg_data.D_ls;
      obj.V = struct('S', S, 's1', s1);
      obj.qstar = x0(1:biped.getNumPositions());
      obj.c = [];
      obj.comtraj = mxDeserialize(msg_data.comtraj);
      obj.mu = double(msg_data.mu);
      obj.gain_set = char(msg_data.gain_set);
      obj.duration = obj.support_times(end) - obj.support_times(1) - 0.001;
    end
  end
end

