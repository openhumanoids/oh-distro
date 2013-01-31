function dircol_test()

options.floating = false;
r = RigidBodyManipulator('../../../../ros_workspace/mit_drcsim_scripts/models/mit_wheeled_stick_torso/model.sdf', options);
% r = TimeSteppingRigidBodyManipulator('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', 0.01, options);

nx = r.getNumStates();

robot_name = 'wheeled_atlas';
options = struct('num_knot_points', 11);
time_to_goal = 1;
goal_dist = 1;

joint_names = r.getStateFrame.coordinates(1:nx/2);

lcmcoder = JLCMCoder(RobotStateCoder(robot_name, joint_names, r.has_floating_base));
joint_names
state_listener = LCMCoordinateFrameWCoder(robot_name, nx, r.getStateFrame().prefix, lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

plan_publisher = RobotPlanPublisher(robot_name, joint_names, r.hrunas_floating_base, 'CANDIDATE_ROBOT_PLAN', options.num_knot_points);



% traj_options.method='implicitdirtran';
traj_options.method='dircol';
traj_options.warning=0;
traj_options.snopt_major_iter_lim = 3;

x0 = [];
while isempty(x0)
  [x0, t0] = state_listener.getNextMessage(1);
end
% t0 = 0;
% x0 = r.getInitialState();
traj_options.trajectory_cost_fun = @(t, x, u) publish_dircol_traj(t, x, u, t0, plan_publisher);
goal_ndx = 5;
xf = x0 + [zeros(goal_ndx - 1, 1); goal_dist; zeros(length(x0) - goal_ndx, 1)];
xf_tol = [inf(goal_ndx - 1, 1); 0; inf(length(x0) - goal_ndx, 1)];
vel_lim = cell2mat({r.body(2:end).velocity_limit})';
pos_lim_max = cell2mat({r.body(2:end).joint_limit_max})';
pos_lim_min = cell2mat({r.body(2:end).joint_limit_min})';
con.x.lb = [pos_lim_min; -vel_lim];
con.x.ub = [pos_lim_max; vel_lim];
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf - xf_tol;
con.xf.ub = xf + xf_tol;
con.xf.c = @(x) end_eff_goal(r, x(1:end/2), r.body(8), [0;0;0],[-0.5478;0.2685;0.6122]);
con.T.lb = .8 * time_to_goal;   
con.T.ub = 1.2 * time_to_goal;
current_utraj = PPTrajectory(foh(linspace(0, time_to_goal, options.num_knot_points),...
  randn(r.getNumInputs(), options.num_knot_points)));
disp('starting trajopt')
[current_utraj, current_xtraj, info] = trajectoryOptimization(r, @cost, @finalcost, x0, current_utraj, con, traj_options);
% v = r.constructVisualizer();
% v.playback(current_xtraj, struct('slider', 'true'));
disp('finished')
while (1)
  [x0, t0] = state_listener.getNextMessage(1);
  if (~isempty(x0))
    traj_options.trajectory_cost_fun = @(t, x, u) publish_dircol_traj(t, x, u, t0, plan_publisher);
    traj_options.xtraj0 = current_xtraj;
    traj_options.xtape0 = 'input';
    [current_utraj, current_xtraj, info] = trajectoryOptimization(r, @cost, @finalcost, x0, current_utraj, con, traj_options);
%     planned_states = repmat(x0, 1, options.num_knot_points);
%     planned_states(goal_ndx,:) = linspace(x0(goal_ndx), x0(goal_ndx) + goal_dist, options.num_knot_points);
%     plan_publisher.publish(linspace(t0, t0 + time_to_goal, options.num_knot_points), planned_states);
%     x = current_xtraj.eval(current_xtraj.tspan(end));
%     x(5)
%     g = end_eff_goal(r, x(1:end/2), r.body(14), [0;0;0], [-0.5478;0.2685;0.6122])
  end
end

end

function g = cost(t,x,u)
  R = 1;
  g = u'*R*u;
  %g = sum((R*u).*u,1);
  dg = [zeros(1,1+size(x,1)),2*u'*R];
%   dg = zeros(1, 1 + size(x,1) + size(u,1));
end

function [h,dh] = finalcost(t,x)
  h = t;
  dh = [1,zeros(1,size(x,1))];
end
    
function [knots, t] = get_knot_points(xtraj, num_knot_points)
  t = linspace(xtraj.tspan(1), xtraj.tspan(2), num_knot_points);
  knots = xtraj.eval(t);
end

function [J, dJ] = publish_dircol_traj(t, x, u, t0, plan_publisher)
  plan_publisher.publish(t + t0, x);
  J=0;
  dJ=[0*t(:);0*x(:);0*u(:)]';
end

function [g] = end_eff_goal(r, q, link, link_pt, goal)
  kindsol = r.doKinematics(q, true);
  [pt1, J1, dJ1] = r.forwardKin(kindsol, link, link_pt);
  g = sum((goal - pt1) .^ 2);
end
  