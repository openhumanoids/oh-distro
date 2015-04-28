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

      obj.mex_ptr = constructRecoveryMexPointer(robot.getMexModelPtr(), qstar, V.S); 

      % Make sure the mex file gets loaded
      obj.publishQPControllerInput();
    end


    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)
      t0 = tic();
      obj.publishQPControllerInput(t_global, x, rpc, contact_force_detected);
      fprintf(1, 'publish recovery input: %fs\n', toc(t0));
      qp_input = [];
    end
  end
end

