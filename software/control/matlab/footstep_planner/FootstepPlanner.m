classdef FootstepPlanner < DRCPlanner
  properties
    biped
    plan_publisher
    lc
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      robot_name = 'atlas';
      obj = obj@DRCPlanner('NAV_GOAL_TIMED',JLCMCoder(NavGoalCoder(robot_name)));
      
      obj.biped = biped;
      obj.biped = enableIdealizedPositionControl(obj.biped, true);
      obj.biped = compile(obj.biped);

      nx = obj.biped.getNumStates();
      joint_names = obj.biped.getStateFrame.coordinates(1:nx/2);
      obj.plan_publisher = FootstepPlanPublisher('CANDIDATE_FOOTSTEP_PLAN');
      
      obj = addInput(obj,'x0','EST_ROBOT_STATE',obj.biped.getStateFrame.lcmcoder,true,true);
    end
    
    function plan(obj,navgoal,data)
      options.plotting = true;
      options.interactive = true;
      options.heel_toe = false;
      [Xright, Xleft] = obj.biped.planFootsteps(data.x0, navgoal, options);
      obj.plan_publisher.publish([Xright, Xleft]);
    end
  end
end


