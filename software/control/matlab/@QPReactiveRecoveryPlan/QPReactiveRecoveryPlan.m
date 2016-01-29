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
      qstar = S.xstar(1:robot.getNumPositions());
      [~, V, ~, ~] = robot.planZMPController([0;0], qstar);

      rpc = robot.getRobotPropertyCache();
      obj.mex_ptr = constructRecoveryMexPointer(robot.getMexModelPtr(), qstar, V.S, rpc); 

      % Make sure the mex files get loaded
      obj.resetInitialization();
      obj.publishQPControllerInput();
    end


    qp_input = getQPControllerInput(obj, t_global, x, contact_force_detected);
  end
end

