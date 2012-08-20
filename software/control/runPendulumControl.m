function runPendulumControl
% swing up and stabilize the pendulum
r = PlanarRigidBodyManipulator('../../ros_workspace/atlas_description/urdf/Pendulum.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = 0.05;

x0 = Point(r.getStateFrame());
xf = x0;
xf.theta = pi;
xf.thetadot = 0;
x0 = double(x0);
xf = double(xf);
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
tf0 = 1;
con.T.lb = 0.5;
con.T.ub = 2;

options.method = 'dircol';
N = 11;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),zeros(r.getNumInputs(),N)));
options.MajorIterationsLimit = 50;
[utraj,xtraj,info] = trajectoryOptimization(r,@cost,@finalcost,x0,utraj0,con,options);
utraj = utraj.setOutputFrame(r.getInputFrame());
options = struct('TSPAN',[0 5]);
runDRCControl(utraj,'pendulum','TRUE_ROBOT_STATE','ACTUATOR_CMDS',options);
keyboard;
end

function [g,dg] = cost(t,x,u);
R = 1;
g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
end

function [h,dh] = finalcost(t,x)
h = t;
dh = [1,zeros(1,size(x,1))];
end