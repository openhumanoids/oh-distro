function runCompassGaitControl
options = struct('view','right','floating',true);
%m = PlanarRigidBodyModel('../../ros_workspace/mit_drcsim_scripts/models/mit_compassGait_robot/compass_gait.urdf',options);
% r = PlanarRigidBodyManipulator(m);
% v = r.constructVisuzlizer();
r = PlanarRigidBodyManipulator('../../ros_workspace/mit_drcsim_scripts/models/mit_compassGait_robot/compass_gait.urdf');
% m3D = RigidBodyModel.parseURDF('../../ros_workspace/mit_drcsim_scripts/models/mit_compassGait_robot/compass_gait.urdf');
m3D = RigidBodyModel('../../ros_workspace/mit_drcsim_scripts/models/mit_compassGait_robot/compass_gait.urdf',struct('floating',true));
r3D = RigidBodyManipulator(m3D);
m2D = PlanarRigidBodyModel('../../ros_workspace/mit_drcsim_scripts/models/mit_compassGait_robot/compass_gait.urdf',struct('floating',true,'view','right'));
r2D = PlanarRigidBodyManipulator(m2D);
options.tspan = [0,inf];
options.sys2D = r2D;
keyboard;
runDRCPlanning(r3D,@runDirtran,'CompassGait','TRUE_ROBOT_STATE','NAV_GOAL_TIMED','ROBOT_PLAN',options);
keyboard
end

function navState2D = projectNavGoal(sys2D,navGoal,comCurr)
%navGoal2D is just the translation in the x direction of the robot body
%frame.
navGoal = navGoal(:);
comCurr = comCurr(:);
currRPY = comCurr(4:6);
currRotMat = rpy2mat(currRPY(1),currRPY(2),currRPY(3));
currOrientation = currRotMat*[1;0;0]; % I suppose the robot always facing the positive x direction in its own frame
% Then I need to project currOrientation to xy 2D plane
currOrientation = currOrientation-[0;0;1]'*currOrientation*[0;0;1];
currOrientation = currOrientation/norm(currOrientation);
navGoalRelTranslation = navGoal(1:3)-comCurr(1:3);
navGoalProj = comCurr(1:3)+sum(navGoalRelTranslation.*currOrientation)*...
    currOrientation; % This is the navigation goal projected to the current orientation in the xy plane
navState2D = zeros(sys2D.getNumContStates(),1);
navState2D(1) = navGoalProj(1);
end

function x2D = state3Dto2D(sys2D,sys3D,x3D)
state3D = sys3D.getStateFrame.coordinates;
state2D = sys2D.getStateFrame.coordinates;
[~,ind2D,ind3D] = intersect(state2D,state3D);
x2D = zeros(sys2D.getNumContStates(),1);
x2D(ind2D) = x3D(ind3D);
end

function [tbreaks,xbreaks,utraj2D,xtraj2D,ltraj2D] = runDirtran(sys3D,navGoal,xCurr,options)
% both navGoal and xCurr are states in 3D
if(isfield(options,'sys2D'))
    sys2D = options.sys2D;
end
xCurr2D = state3Dto2D(sys2D,sys3D,xCurr);
comCurr = xCurr(1:6);
navState2D = projectNavGoal(sys2D,navGoal,comCurr);
clear mex
nq = sys2D.num_q;
nu = sys2D.getNumInputs();
nL = sys2D.getNumJointLimits();
nC = sys2D.num_contacts;
N = 10;
x0 = xCurr2D;
xf = navState2D;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = navState2D;
con.xf.ub = navState2D;
con.xf.lb(2) = 0;
con.xf.ub(2) = inf;
con.xf.lb(nq+1:end) = -inf(nq,1);
con.xf.ub(nq+1:end) = inf(nq,1);
tf0 = 2;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),randn(nu,N)));
xvec = repmat(x0,1,N)+(xf-x0)*linspace(0,1,N);
if isfield(options,'xtraj_init')
    traj0.x = options.xtraj_init;
    traj0.lambda = options.ltraj_init;
    utraj0 = options.utraj_init;
else
traj0.x = PPTrajectory(foh(linspace(0,tf0,N),xvec));
traj0.lambda = PPTrajectory(foh(linspace(0,tf0,N),zeros(nL,N)));
traj0.x = traj0.x.setOutputFrame(sys2D.getStateFrame);
end
con.fixtime = 0;
con.T.ub = 4;
con.T.lb = 0.5;
con.u.lb = sys2D.umin;
con.u.ub = sys2D.umax;
con.noflight = 1;

if(isfield(options,'alphamult'))
    con.alphamult = options.alphamult;
    con.betamult = options.betamult;
else
con.alphamult = 10;
con.betamult = 10;
end

options.traj0 = traj0;

options.method='implicitdirtran';
options.MajorIterationsLimit = 1000;
options.IterationsLimit = 1e8;
options.VerifyLevel = 0;
options.SuperbasicsLimit = 3000;
snprint('snopt.out');

tic
[utraj2D,xtraj2D,info] = sys2D.trajectoryOptimization(@cost,@finalcost,x0,utraj0,con,options);
toc

ltraj2D = xtraj2D.ltraj;
xtraj2D = xtraj2D.xtraj;
tbreaks = xtraj2D.getBreaks();
xbreaks2D = xtraj2D.eval(tbreaks);
state3D = sys3D.getStateFrame.coordinates;
state2D = sys2D.getStateFrame.coordinates;
[~,ind2D,ind3D] = intersect(state2D,state3D);
xbreaks3D = repmat(xCurr,1,length(tbreaks));
xbreaks3D(ind3D,:) = xbreaks2D(ind2D,:);
xbreaks = reshape(xbreaks3D,[],1);
end

function [g,dg] = cost(dt,x,u,sys)
nX = sys.getNumContStates();
Q = zeros(nX);
nU = sys.getNumInputs();
R = eye(nU);

g = dt*sum((R*u).*u,1);
dg = [g/dt zeros(1,nX) 2*(R*u)'*dt];
end

function [h,dh] = finalcost(t,x)
h = 0;
dh = [0 zeros(1,length(x))];
end