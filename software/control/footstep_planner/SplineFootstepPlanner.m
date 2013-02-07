classdef SplineFootstepPlanner
% A planner which tries to follow a cubic spline from the robot's current position to its goal position and orientation
  properties
    manip
    step_length
    step_width
    step_time
    step_rot
    state_listener
    nav_goal_listener
    v
    xstar
    plan_publisher
  end
  
  methods
    function obj = SplineFootstepPlanner(r, step_length, step_width, step_rot, step_time)
      % @param r the time-stepping manipulator
      % @param step_length the length of each step in m
      % @param step_time  the duration of each step in s

      typecheck(r, 'TimeSteppingRigidBodyManipulator');
      typecheck(step_length, 'double');
      typecheck(step_time, 'double');

      obj.manip = r;
      obj.manip = enableIdealizedPositionControl(r, true);
      obj.manip = compile(obj.manip);

      obj.step_length = step_length;
      obj.step_width = step_width;
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
          start_pos = getAtlasFeetPos(obj.manip, q0);
          poses = [start_pos, g];
          traj = turnGoTraj(poses);
          [lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, obj.step_length, obj.step_width, obj.step_rot);
          figure(21)
          plotFootstepPlan(traj, lambda, ndx_r, ndx_l, obj.step_width)
          Xright = footstepLocations(traj, lambda(ndx_r), -pi/2, obj.step_width);
          Xleft = footstepLocations(traj, lambda(ndx_l), pi/2, obj.step_width);
          
          [zmptraj, lfoottraj, rfoottraj, ts] = planZMPandFootTrajectory(obj.manip, q0, Xright, Xleft, obj.step_time);

          %% covert ZMP plan into COM plan using LIMP model
          addpath(fullfile(getDrakePath,'examples','ZMP'));
          [com,Jcom] = getCOM(obj.manip,q0);
          comdot = Jcom*obj.xstar(getNumDOF(obj.manip)+(1:getNumDOF(obj.manip)));
          % comdot = Jcom*x0(getNumDOF(obj.manip)+(1:getNumDOF(obj.manip)));
          limp = LinearInvertedPendulum(com(3,1));

          comtraj = [ ZMPplanner(limp,com(1:2),comdot(1:2),setOutputFrame(zmptraj,desiredZMP)); ...
            ConstantTrajectory(com(3,1)) ];

          %% compute joint positions with inverse kinematics

          ind = getActuatedJoints(obj.manip);
          rfoot_body = obj.manip.findLink('r_foot');
          lfoot_body = obj.manip.findLink('l_foot');

          cost = Point(obj.manip.getStateFrame,1);
          cost.pelvis_x = 0;
          cost.pelvis_y = 0;
          cost.pelvis_z = 0;
          cost.pelvis_roll = 1000;
          cost.pelvis_pitch = 1000;
          cost.pelvis_yaw = 0;
          cost.back_mby = 100;
          cost.back_ubx = 100;
          cost = double(cost);
          options = struct();
          options.Q = diag(cost(1:obj.manip.getNumDOF));
          options.q_nom = q0;

          disp('computing ik...')
          for i=1:length(ts)
            t = ts(i);
            if (i>1)
              q(:,i) = inverseKin(obj.manip,q(:,i-1),0,comtraj.eval(t),rfoot_body,rfoottraj.eval(t),lfoot_body,lfoottraj.eval(t),options);
            else
              q = q0;
            end
            q_d(:,i) = q(ind,i);
            obj.v.draw(t,q(:,i));
          end

          if (1)
            %% to view the motion plan:
            xtraj = setOutputFrame(PPTrajectory(spline(ts,[q;0*q])),getOutputFrame(obj.manip));
            obj.plan_publisher.publish(ts, xtraj.eval(ts));
            % playback(v,xtraj,struct('slider',true));

            % continue;
          end



        end
      end
    end
  end

end


