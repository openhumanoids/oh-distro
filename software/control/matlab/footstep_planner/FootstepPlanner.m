classdef FootstepPlanner < DRCPlanner
  properties
    biped
    plan_publisher
  end
  
  methods
    function obj = FootstepPlanner(biped)
      typecheck(biped, 'Biped');
      
      robot_name = 'atlas';
      obj = obj@DRCPlanner('NAV_GOAL_TIMED',JLCMCoder(NavGoalCoder(robot_name)));
      
      obj.biped = biped;
      obj.biped.manip = enableIdealizedPositionControl(obj.biped.manip, true);
      obj.biped.manip = compile(obj.biped.manip);

      nx = obj.biped.manip.getNumStates();
      joint_names = obj.biped.manip.getStateFrame.coordinates(1:nx/2);
      obj.plan_publisher = RobotPlanPublisher(robot_name, joint_names, true, 'CANDIDATE_ROBOT_PLAN');
      
      obj = addInput(obj,'x0','TRUE_ROBOT_STATE',obj.biped.manip.getStateFrame.lcmcoder,true);
    end
    
    function p = plan(obj,navgoal,data)
      obj.biped.walkingPlan(data.x0,navgoal,struct('plotting', true,'interactive', true));
    end
  end
end


