classdef SplineFootstepPlanner
% A planner which tries to follow a cubic spline from the robot's current position to its goal position and orientation
  properties
    manip
    step_length
    step_time
    state_listener
    nav_goal_listener
  end
  
  methods
    function obj = SplineFootstepPlanner(r, step_length, step_time)
      % @param r the time-stepping manipulator
      % @param step_length the length of each step in m
      % @param step_time  the duration of each step in s

      typecheck(r, 'TimeSteppingRigidBodyManipulator');
      typecheck(step_length, 'double');
      typecheck(step_time, 'double');

      obj.manip = r;
      obj.step_length = step_length;
      obj.step_time = step_time;

      nx = r.getNumStates();
      robot_name = 'atlas';
      joint_names = r.getStateFrame.coordinates(1:nx/2);
      lcmcoder = JLCMCoder(RobotStateCoder(robot_name, joint_names));
      obj.state_listener = LCMCoordinateFrameWCoder(robot_name, nx, r.getStateFrame().prefix, lcmcoder);
      obj.state_listener.subscribe('EST_ROBOT_STATE');

      obj.nav_goal_listener = RobotNavGoalListener(robot_name, 'NAV_GOAL_TIMED');
    end

    function run(obj)
      x0 = [];
      while 1
        [x, t] = obj.state_listener.getNextMessage(1);
        if ~isempty(x)
          x0 = x;
          t0 = t;
        end
        g = obj.nav_goal_listener.getNextMessage(0);
        if ~isempty(g) && ~isempty(x0)
          x0(1:6)
          g
          [zmptraj, lfoottraj, rfoottraj, ts] = planZMPandFootTrajectory(obj.manip, x0(1:(end/2)), g, obj.step_length, obj.step_time);
        end
      end
    end
  end

end


