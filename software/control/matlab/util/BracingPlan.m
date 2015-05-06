classdef BracingPlan < QPControllerPlanMatlabImplementation
  methods
    function obj = BracingPlan(r)
      d = load(r.bracing_config_file);
      qstar = d.xstar(1:r.getNumPositions());
      qp_input = atlasControllers.QPInputConstantHeight();
      qp_input.whole_body_data.q_des = qstar;
      qp_input.param_set_name = 'bracing';
      obj.default_qp_input_ = qp_input;

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
