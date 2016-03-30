classdef SilentPlan < QPControllerPlanMatlabImplementation  
  properties
    q_des
  end

  methods
    function obj = SilentPlan(robot)
      obj.q_des = zeros(robot.getNumPositions(), 1);
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = drake.lcmt_qp_controller_input;
      qp_input.zmp_data = drake.lcmt_zmp_data;
      qp_input.whole_body_data = drake.lcmt_whole_body_data;
      qp_input.whole_body_data.q_des = obj.q_des;
      qp_input.param_set_name = '';
      qp_input.be_silent = true;
    end
  end
end
