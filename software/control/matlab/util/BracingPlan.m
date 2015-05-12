classdef BracingPlan < QPControllerPlanMatlabImplementation
  methods
    function obj = BracingPlan(r, q_des)
      if (nargin < 2)
        d = load(r.bracing_config_file);
        q_des = d.xstar(1:r.getNumPositions());
      end
      qp_input = atlasControllers.QPInputConstantHeight();
      qp_input.whole_body_data.q_des = q_des;
      qp_input.param_set_name = 'bracing';
      obj.default_qp_input_ = qp_input.to_lcm();

      obj.duration_ = inf;

    end

    function qp_input = getQPControllerInput(obj, t, varargin)
      if isempty(obj.start_time)
        obj.start_time = t;
      end
      qp_input = obj.default_qp_input_;
    end
  end
end
