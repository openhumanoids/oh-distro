classdef SilentPlan < QPControllerPlanMatlabImplementation
  methods
    function obj = SilentPlan(robot)
      obj.default_qp_input_ = bipedControllers.QPInputConstantHeight();
      obj.default_qp_input_.be_silent = true;
      obj.default_qp_input_.whole_body_data.q_des = zeros(robot.getNumPositions(), 1);
      obj.default_qp_input_ = obj.default_qp_input_.to_lcm();
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = obj.default_qp_input_;
    end
  end
end
