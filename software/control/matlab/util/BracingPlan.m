classdef BracingPlan < QPControllerPlanMatlabImplementation
  properties
    lc
    q_des
  end

  methods
    function obj = BracingPlan(r, q_des)
      if (nargin < 2)
        d = load(r.bracing_config_file);
        q_des = d.xstar(1:r.getNumPositions());
      end
      obj.q_des = q_des;

      obj.duration_ = inf;
      obj.lc = lcm.lcm.LCM.getSingleton();

    end

    function qp_input = getQPControllerInput(obj, t, varargin)
      if isempty(obj.start_time)
        obj.start_time = t;
      end
      qp_input = drake.lcmt_qp_controller_input;
      qp_input.whole_body_data = drake.lcmt_whole_body_data;
      qp_input.whole_body_data.q_des = obj.q_des;
      qp_input.zmp_data = drake.lcmt_zmp_data;
      qp_input.param_set_name = 'bracing';

      pressure_msg = atlas.pump_command_t;
      pressure_msg.desired_psi = 1000;
      pressure_msg.desired_rpm = 5000;
      pressure_msg.cmd_max = 60;
      pressure_msg.utime = t * 1e6;
      obj.lc.publish('ATLAS_PUMP_COMMAND', pressure_msg);
    end
  end
end
