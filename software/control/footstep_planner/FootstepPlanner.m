classdef FootstepPlanner
  properties
    biped
    state_listener
    nav_goal_listener
    plan_publisher
  end
  
  methods
    function obj = FootstepPlanner(biped)

      typecheck(biped, 'Biped');
      
      obj.biped = biped;
      obj.biped.manip = enableIdealizedPositionControl(obj.biped.manip, true);
      obj.biped.manip = compile(obj.biped.manip);

      nx = obj.biped.manip.getNumStates();
      robot_name = 'atlas';
      joint_names = obj.biped.manip.getStateFrame.coordinates(1:nx/2);
      lcmcoder = JLCMCoder(RobotStateCoder(robot_name, joint_names));
      obj.state_listener = LCMCoordinateFrameWCoder(robot_name, nx, obj.biped.manip.getStateFrame().prefix, lcmcoder);
      obj.state_listener.subscribe('EST_ROBOT_STATE');

      obj.nav_goal_listener = RobotNavGoalListener(robot_name, 'NAV_GOAL_TIMED');

      obj.plan_publisher = RobotPlanPublisher(robot_name, joint_names, true, 'CANDIDATE_ROBOT_PLAN');
    end

    function run(obj)
      x0 = [];
      while 1
        [x, t] = obj.state_listener.getNextMessage(1);
        if ~isempty(x)
          x0 = x;
          t0 = t;
          obj.biped.visualizer.draw(t0, x0);
        end
        g = obj.nav_goal_listener.getNextMessage(0);
        if ~isempty(g) && ~isempty(x0)
          [xtraj, ts] = obj.biped.roughWalkingPlan(x0, g);
          obj.plan_publisher.publish(ts, xtraj.eval(ts));
        end
      end
    end
  end
end


