classdef FootstepPlanner
  properties
    manip
    step_length
    step_time
    step_rot
    state_listener
    nav_goal_listener
    v
    xstar
    plan_publisher
  end
  
  methods
    function obj = FootstepPlanner(r, step_length, step_rot, step_time)

      typecheck(r, 'TimeSteppingRigidBodyManipulator');
      typecheck(step_length, 'double');
      typecheck(step_time, 'double');

      obj.manip = r;
      obj.manip = enableIdealizedPositionControl(r, true);
      obj.manip = compile(obj.manip);
      
      

      obj.step_length = step_length;
      obj.step_rot = step_rot;
      obj.step_time = step_time;

      nx = r.getNumStates();
      robot_name = 'atlas';
      joint_names = r.getStateFrame.coordinates(1:nx/2);
      lcmcoder = JLCMCoder(RobotStateCoder(robot_name, joint_names));
      obj.state_listener = LCMCoordinateFrameWCoder(robot_name, nx, r.getStateFrame().prefix, lcmcoder);
      obj.state_listener.subscribe('EST_ROBOT_STATE');

      obj.nav_goal_listener = RobotNavGoalListener(robot_name, 'NAV_GOAL_TIMED');
      obj.v = obj.manip.constructVisualizer();
      load('../data/atlas_fp.mat');
      obj.xstar = xstar;

      obj.plan_publisher = RobotPlanPublisher(robot_name, joint_names, true, 'CANDIDATE_ROBOT_PLAN');
    end

    function run(obj)
      x0 = [];
      while 1
        [x, t] = obj.state_listener.getNextMessage(1);
        if ~isempty(x)
          x0 = x;
          t0 = t;
          obj.v.draw(t0, x0);
        end
        g = obj.nav_goal_listener.getNextMessage(0);
        if ~isempty(g) && ~isempty(x0)
          x0(1:6)
          q0 = x0(1:end/2);
          [start_pos, step_width] = getAtlasFeetPos(obj.manip, q0);
          poses = [start_pos, g];
          traj = turnGoTraj(poses);
          [lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, obj.step_length, step_width, obj.step_rot);
          figure(21)
          Xright = footstepLocations(traj, lambda(ndx_r), -pi/2, step_width);
          Xleft = footstepLocations(traj, lambda(ndx_l), pi/2, step_width);
          plotFootstepPlan(traj, Xright, Xleft);
          drawnow
          
          [lambda, Xright, Xleft] = optimizeFootsteps(traj, lambda, obj.step_length, step_width, obj.step_rot, ndx_r, ndx_l);
          figure(22)
          plotFootstepPlan(traj, Xright, Xleft);
          
          [zmptraj, lfoottraj, rfoottraj, ts] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);

          xtraj = computeZMPPlan(obj.manip, obj.v, x0, zmptraj, lfoottraj, rfoottraj, ts);
          obj.plan_publisher.publish(ts, xtraj.eval(ts));
        end
      end
    end
  end
end


