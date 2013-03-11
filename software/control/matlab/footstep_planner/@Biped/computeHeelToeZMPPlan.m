function xtraj = computeHeelToeZMPPlan(biped, x0, zmptraj, foottraj, contact_ref, ts)

q0 = x0(1:end/2);
%% covert ZMP plan into COM plan using LIMP model
addpath(fullfile(getDrakePath,'examples','ZMP'));
[com,Jcom] = getCOM(biped,q0);
comdot = Jcom*x0(getNumDOF(biped)+(1:getNumDOF(biped)));
limp = LinearInvertedPendulum(com(3,1));

comtraj = [ ZMPplanner(limp,com(1:2),comdot(1:2),setOutputFrame(zmptraj,desiredZMP)); ...
  ConstantTrajectory(com(3,1)) ];

%% compute joint positions with inverse kinematics

ind = getActuatedJoints(biped);
foot_body = struct('right', biped.findLink(biped.r_foot_name), 'left', biped.findLink(biped.l_foot_name));

cost = Point(biped.getStateFrame,1);
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
options.Q = diag(cost(1:biped.getNumDOF));
options.q_nom = q0;

visualizer = biped.constructVisualizer();

disp('computing ik...')
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    cons = {biped,q(:,i-1),0,comtraj.eval(t)};
    for f = {'right', 'left'}
      foot = f{1};
      for g = {'heel', 'toe'}
        grp = g{1};
        cons{end+1} = foot_body.(foot);
        cons{end+1} = contact_ref.(foot).(grp);
        cons{end+1} = struct('min', foottraj.(foot).(grp).lb.eval(t),...
                             'max', foottraj.(foot).(grp).ub.eval(t));
      end
    end
    q(:,i) = inverseKin(cons{:},options);
  else
    q = q0;
  end
  q_d(:,i) = q(ind,i);
  visualizer.draw(t,q(:,i));
end

xtraj = setOutputFrame(PPTrajectory(spline(ts,[q;0*q])),getOutputFrame(biped));