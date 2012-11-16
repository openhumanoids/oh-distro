function runAtlasReachingExample

r = RigidBodyManipulator('../../ros_workspace/deprecated/atlas_description/urdf/atlas_robot.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame());
xf = x0;
xf.RShoulderYaw = .4;
xf.RShoulderPitch = -.8;
xf.RElbowPitch = -1.5;

x0 = double(x0);
xf = double(xf);
tf0 = 4;

con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf;
con.xf.ub = xf;
con.T.lb = 2;
con.T.ub = 6;

options.method='dircol';
      
  function [g,dg] = cost(t,x,u);
    R = 1;
    g = sum((R*u).*u,1);
    dg = [zeros(1,1+size(x,1)),2*u'*R];
  end
      
  function [h,dh] = finalcost(t,x)
    h = t;
    dh = [1,zeros(1,size(x,1))];
  end
      
utraj0 = PPTrajectory(foh(linspace(0,tf0,5),zeros(r.getNumInputs(),5)));
tic
%options.grad_test = true;
options.MajorIterationsLimit = 5;
options.MinorIterationsLimit = 5;
[utraj,xtraj,info] = trajectoryOptimization(r,@cost,@finalcost,x0,utraj0,con,options);
info
toc


v.playback(xtraj);

end