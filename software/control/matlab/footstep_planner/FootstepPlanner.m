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
      obj.biped = enableIdealizedPositionControl(obj.biped, true);
      obj.biped = compile(obj.biped);

      nx = obj.biped.getNumStates();
      joint_names = obj.biped.getStateFrame.coordinates(1:nx/2);
      obj.plan_publisher = FootstepPlanPublisher(robot_name, 'r_foot','l_foot', 'CANDIDATE_FOOTSTEP_PLAN');
      
      obj = addInput(obj,'x0','TRUE_ROBOT_STATE',obj.biped.getStateFrame.lcmcoder,true,true);
      obj = addInput(obj,'con', 'TRAJ_OPT_CONSTRAINT', drc.traj_opt_constraint_t, false, true);
    end
    
    function plan(obj,navgoal,data)
      options.plotting = true;
      options.interactive = true;
      options.heel_toe = false;
      [Xright, Xleft] = obj.biped.planFootsteps(data.x0, navgoal, options);
      
      tright = Xright(7,:);
      tleft = Xleft(7,:);
      
      % note: assumes right foot comes first
%       tright = [0,((2:size(Xright,2))-1.5)*obj.biped.step_time];
%       tleft = ((1:size(Xleft,2))-1)*obj.biped.step_time;
      
      obj.plan_publisher.publish(tright,Xright(1:6,:),tleft,Xleft(1:6,:));
    end
  end
end


