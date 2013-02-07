function xtraj = computeZMPPlan(r, v, x0, zmptraj, lfoottraj, rfoottraj, ts)

q0 = x0(1:end/2);
%% covert ZMP plan into COM plan using LIMP model
addpath(fullfile(getDrakePath,'examples','ZMP'));
[com,Jcom] = getCOM(r,q0);
% comdot = Jcom*obj.xstar(getNumDOF(r)+(1:getNumDOF(r)));
comdot = Jcom*x0(getNumDOF(r)+(1:getNumDOF(r)));
limp = LinearInvertedPendulum(com(3,1));

comtraj = [ ZMPplanner(limp,com(1:2),comdot(1:2),setOutputFrame(zmptraj,desiredZMP)); ...
  ConstantTrajectory(com(3,1)) ];

%% compute joint positions with inverse kinematics

ind = getActuatedJoints(r);
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

cost = Point(r.getStateFrame,1);
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
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q0;

disp('computing ik...')
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,comtraj.eval(t),rfoot_body,rfoottraj.eval(t),lfoot_body,lfoottraj.eval(t),options);
  else
    q = q0;
  end
  q_d(:,i) = q(ind,i);
  v.draw(t,q(:,i));
end

xtraj = setOutputFrame(PPTrajectory(spline(ts,[q;0*q])),getOutputFrame(r));