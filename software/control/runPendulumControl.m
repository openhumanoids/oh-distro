function runPendulumControl
% swing up and stabilize the pendulum
r = PlanarRigidBodyManipulator('../../ros_workspace/atlas_description/urdf/Pendulum.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = 0.05;

x0_pt = Point(r.getStateFrame());
xf_pt = x0_pt;
xf_pt.theta = pi;
xf_pt.thetadot = 0;
x0 = double(x0_pt);
xf = double(xf_pt);
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
uf_pt = Point(r.getInputFrame());
uf_pt.theta = 0;
uf = double(uf_pt);
con.uf.lb = uf;
con.uf.ub = uf;
tf0 = 1;
con.T.lb = 0.5;
con.T.ub = 2;

options.method = 'dircol';
N = 11;
utraj0 = PPTrajectory(foh(linspace(0,tf0,N),zeros(r.getNumInputs(),N)));
options.MajorIterationsLimit = 50;
[utraj,xtraj,info] = trajectoryOptimization(r,@cost,@finalcost,x0,utraj0,con,options);
% utraj = utraj.setOutputFrame(r.getInputFrame());
% utraj = utraj.setInputFrame(r.getOutputFrame());
[ltvsys,V] = tvlqr(r,xtraj,utraj,eye(2),1,eye(2));
c_tvlqr_y0 = FunctionHandleTrajectory(@(t) utraj.eval(t)-ltvsys.D.eval(t)*xtraj.eval(t),[1 1],utraj.getBreaks());
c_tvlqr = AffineSystem([],[],[],[],[],[],[],ltvsys.D,c_tvlqr_y0);
c_tvlqr = c_tvlqr.setOutputFrame(r.getInputFrame());
c_tvlqr = c_tvlqr.setInputFrame(r.getOutputFrame());
[c_tilqr,V_tilqr] = tilqr(r,xf_pt,uf_pt,eye(2),1);
c_tilqr = AffineSystem([],[],[],[],[],[],[],c_tilqr.D,-c_tilqr.D*xf);
c_tilqr = c_tilqr.setOutputFrame(r.getInputFrame());
c_tilqr = c_tilqr.setInputFrame(r.getOutputFrame());
% c = HybridDrakeSystem(2,1);
% c = c.setInputFrame(c_tvlqr.getInputFrame());
% c = c.setOutputFrame(c_tvlqr.getOutputFrame());
% [c,mode1] = c.addMode(c_tvlqr,'tvlqr');
% [c,mode2] = c.addMode(c_tilqr,'tilqr');
% c = c.addTransition(mode1,@(obj,t,x,u)tilqr_guard(obj,t,x,u,V_tilqr),...
%     @(obj,t,x,u) tilqr_transition(obj,mode1,t,x,u,mode2),false,true,mode2);
% ltvsys = ltvsys.setInputFrame(r.getOutputFrame());
% ltvsys = ltvsys.setOutputFrame(r.getInputFrame());
options1 = struct('tspan',utraj.tspan);
runDRCControl(c_tvlqr,'Pendulum','TRUE_ROBOT_STATE','ACTUATOR_CMDS',options1);
options2 = struct('tspan',[utraj.tspan(2) inf]);
runDRCControl(c_tilqr,'Pendulum','TRUE_ROBOT_STATE','ACTUATOR_CMDS',options2);
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

function g = tilqr_guard(obj,t,x,u,V_tilqr)
g = (u-[pi;0])'*V_tilqr*(u-[pi;0])-1;
end

function [xp,mode,status] = tilqr_transition(obj,mode,t,xm,u,to_mode)
xp = xm;
mode = to_mode;
status = 0;
end