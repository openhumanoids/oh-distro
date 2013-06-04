function [xtraj, qtraj, htraj, ts] = robotWalkingPlan(biped, q0, qstar, zmptraj, comtraj, link_constraints)

% time spacing of samples for IK
ts = 0:0.1:zmptraj.tspan(end);
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,zmptraj.tspan(end),300);
end

%% create desired joint trajectory
cost = Point(biped.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_lbz = 10;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:biped.getNumDOF));
%   options.q_nom = q0;
options.q_nom = qstar;
% [options.jointLimitMin, options.jointLimitMax] = biped.getJointLimits();
% joint_names = biped.getStateFrame.coordinates(1:biped.getNumDOF());
% knee_ind = find(~cellfun(@isempty,strfind(joint_names,'kny')));
% options.jointLimitMin(knee_ind) = 0.6;

rfoot_body = biped.findLink(biped.r_foot_name);
lfoot_body = biped.findLink(biped.l_foot_name);

msg ='Walk Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
htraj = [];
full_IK_calls = 0;
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    approx_args = {};
    for j = 1:length(link_constraints)
      if ~isempty(link_constraints(j).traj)
        approx_args(end+1:end+3) = {link_constraints(j).link_ndx, link_constraints(j).pt, link_constraints(j).traj.eval(t)};
      else
        pos_min = link_constraints(j).min_traj.eval(t);
        pos_max = link_constraints(j).max_traj.eval(t);
        approx_args(end+1:end+3) = {link_constraints(j).link_ndx, link_constraints(j).pt, struct('min', pos_min, 'max', pos_max)};
      end
    end
    try
      q(:,i) = approximateIK(biped,q(:,i-1),0,[comtraj.eval(t);nan],approx_args{:},options);
    catch err
      disp(err)
      full_IK_calls = full_IK_calls + 1
      q(:,i) = inverseKin(biped,q(:,i-1),0,[comtraj.eval(t);nan],approx_args{:},options);
    end

  else
    q = q0;
  end
  com = getCOM(biped,q(:,i));
  htraj = [htraj com(3)];
%   v.draw(t,q(:,i));
end
qtraj = PPTrajectory(spline(ts,q));
htraj = PPTrajectory(spline(ts,htraj));
xtraj = zeros(getNumStates(biped),length(ts));
xtraj(1:getNumDOF(biped),:) = q;

