classdef FootstepPlanner < DRCPlanner
  properties
    biped
    % plan_publisher
    lc
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      robot_name = 'atlas';
      obj = obj@DRCPlanner('NAV_GOAL_TIMED',JLCMCoder(NavGoalCoder(robot_name)));
      
      obj.biped = biped;
%       obj.biped = enableIdealizedPositionControl(obj.biped, true);
%       obj.biped = compile(obj.biped);

      nx = obj.biped.getNumStates();
      joint_names = obj.biped.getStateFrame.coordinates(1:nx/2);
      % obj.plan_publisher = FootstepPlanPublisher('CANDIDATE_FOOTSTEP_PLAN');
      
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame().lcmcoder,true,true);
      obj = addInput(obj, 'plan_con', 'FOOTSTEP_PLAN_CONSTRAINT', drc.footstep_plan_t(), false, true);
      obj = addInput(obj, 'plan_commit', 'COMMITTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true, true);
      obj = addInput(obj, 'plan_reject', 'REJECTED_FOOTSTEP_PLAN', drc.footstep_plan_t(), false, true, true);
    end
    
    function X = plan(obj,navgoal,data)
      navgoal
      %%% HACK for DRC Qual 1 
      navgoal(3) = 0;
      time_limit = navgoal(7);
      isnew = true;
      options.plotting = true;
      options.interactive = true;
      options.heel_toe = false;
      X = obj.biped.optimizeRLFootstepPlan(data.x0, navgoal, @publish, @obj.updateData, data);

      function publish(X, htfun)
        obj.biped.publish_footstep_plan(X, htfun, data.utime, isnew);
        isnew = false;
      end

    end
  end
end


