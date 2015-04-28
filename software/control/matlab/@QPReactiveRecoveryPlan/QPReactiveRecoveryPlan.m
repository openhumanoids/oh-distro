classdef QPReactiveRecoveryPlan < QPControllerPlanMatlabImplementation
  properties
    mex_ptr;
  end

  methods
    function obj = QPReactiveRecoveryPlan(robot, options)
      checkDependency('iris');
      if nargin < 2
        options = struct();
      end
      S = load(robot.fixed_point_file);
      qtraj = S.xstar(1:robot.getNumPositions());
      [~, V, ~, ~] = robot.planZMPController([0;0], qtraj);

      obj.mex_ptr = constructRecoveryMexPointer(robot.getMexModelPtr(), V.S); 
    end


    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)
      obj.publishQPControllerInput(t_global, x, rpc, contact_force_detected);
      qp_input = [];
    end
  end
end

