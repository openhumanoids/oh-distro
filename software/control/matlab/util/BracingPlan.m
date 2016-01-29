classdef BracingPlan < QPControllerPlanMatlabImplementation
  properties
    lc
  end

  methods
    function obj = BracingPlan(r, q_des)
      if (nargin < 2)
        d = load(r.bracing_config_file);
        q_des = d.xstar(1:r.getNumPositions());
      end
      qp_input = bipedControllers.QPInputConstantHeight();
      qp_input.whole_body_data.q_des = q_des;
      qp_input.param_set_name = 'bracing';
      obj.default_qp_input_ = qp_input.to_lcm();

      obj.duration_ = inf;
      obj.lc = lcm.lcm.LCM.getSingleton();

    end

    function qp_input = getQPControllerInput(obj, t, varargin)
      if isempty(obj.start_time)
        obj.start_time = t;
      end
      qp_input = obj.default_qp_input_;

      pressure_msg = drc.atlas_pump_command_t;
      pressure_msg.desired_psi = 1000;
      pressure_msg.desired_rpm = 5000;
      pressure_msg.cmd_max = 60;
      pressure_msg.utime = t * 1e6;
      obj.lc.publish('ATLAS_PUMP_COMMAND', pressure_msg);
    end
  end
end
