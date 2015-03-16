classdef DRCQPLocomotionPlan < QPLocomotionPlan
  methods
    function obj = DRCQPLocomotionPlan(varargin)
      obj = obj@QPLocomotionPlan(varargin{:});
    end
  end

  methods(Static)
    function obj = from_qp_locomotion_plan_t(msg, biped)
      typecheck(biped, 'Biped');
      obj = DRCQPLocomotionPlan(biped);
      obj.robot = biped;
      obj.duration = double(msg.duration);
      obj.start_time = double(msg.start_time);
      obj.x0 = double(msg.x0);
      obj.support_times = double(msg.support_times);
      obj.supports = mxDeserialize(msg.supports);
      obj.link_constraints = mxDeserialize(msg.link_constraints);
      obj.zmptraj = mxDeserialize(msg.zmptraj);
      if isnumeric(obj.zmptraj)
        obj.zmp_final = obj.zmptraj;
      else
        obj.zmp_final = fasteval(obj.zmptraj, obj.zmptraj.tspan(end));
      end
      obj.LIP_height = msg.LIP_height;
      obj.V = struct('S', double(msg.S), 's1', mxDeserialize(msg.s1));
      obj.qtraj = mxDeserialize(msg.qtraj);
      obj.comtraj = mxDeserialize(msg.comtraj);
      obj.mu = double(msg.mu);
      obj.gain_set = char(msg.gain_set);
    end

    function msg = toLCM(obj)
      msg = drc.qp_locomotion_plan_t();
      msg.duration = obj.duration;
      msg.start_time = obj.start_time;

      msg.n_states = numel(obj.x0);
      msg.x0 = obj.x0;

      msg.n_support_times = length(obj.support_times);
      msg.support_times = obj.support_times;

      msg.supports = mxSerialize(obj.supports);
      msg.n_supports_bytes = length(msg.supports);

      msg.link_constraints = mxSerialize(obj.link_constraints);
      msg.n_link_constraints_bytes = length(msg.link_constraints);

      msg.zmptraj = mxSerialize(obj.zmptraj);
      msg.n_zmptraj_bytes = length(msg.zmptraj);

      msg.LIP_height = obj.LIP_height;

      msg.S = obj.V.S;
      msg.s1 = mxSerialize(obj.V.s1);
      msg.n_s1_bytes = length(msg.s1);

      msg.qtraj = mxSerialize(obj.qtraj);
      msg.n_qtraj_bytes = length(msg.qtraj);

      msg.comtraj = mxSerialize(obj.comtraj);
      msg.n_comtraj_bytes = length(msg.comtraj);

      msg.mu = obj.mu;
      msg.gain_set = obj.gain_set;
    end
  end
end
