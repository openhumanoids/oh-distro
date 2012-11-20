function runAtlasControl

options.view = 'right';
options.floating = true;
m = RigidBodyModel('/home/scottk/Atlas/urdf/drc_robot_minimal_contact.urdf',options);
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);
r = setSimulinkParam(r,'MinStep','0.001');
% v = r.constructVisualizer;
% v.display_dt = 0.05;

nx = r.manip.getNumStates();
nq = nx/2;
nu = r.manip.getNumInputs();

% task parameters
Q = diag([100*ones(1,nq) 1*ones(1,nq)]);
R = 0.1*eye(nu);

x0 = Point(r.getStateFrame);
% x0.neck_ay = 0.1;
% x0.l_arm_shx = -1.350000;
% x0.l_arm_ely = 1.570000;
% x0.l_arm_elx = 0.600000;
% x0.r_arm_ely = 1.570000;
% x0.r_arm_elx = -0.600000;
% x0.r_arm_shx = 1.350000;
% x0.l_leg_kny = 0.20000;
% x0.l_leg_uay = -0.100000;
% x0.r_leg_kny = 0.20000;
% x0.r_leg_uay = -0.100000;

xstar = double(x0);
ustar = double(Point(r.getInputFrame));
xstar = r.manip.resolveConstraints(double(xstar));

% [A,B,C,D,xn0] = dlinearize(r,dt,0,xstar,ustar);
% [K,S] = dlqr(A,B,Q,R);
K = randn(nu,nx);
ltisys = LinearSystem([],[],[],[],[],-K);

ltisys = setInputFrame(ltisys,CoordinateFrame([r.getStateFrame.name,' - ', mat2str(xstar,3)],length(xstar),r.getStateFrame.prefix));
r.getStateFrame.addTransform(AffineTransform(r.getStateFrame,ltisys.getInputFrame,eye(length(xstar)),-xstar));
ltisys.getInputFrame.addTransform(AffineTransform(ltisys.getInputFrame,r.getStateFrame,eye(length(xstar)),+xstar));

ltisys = setOutputFrame(ltisys,CoordinateFrame([r.getInputFrame.name,' + ',mat2str(ustar,3)],length(ustar),r.getInputFrame.prefix));
ltisys.getOutputFrame.addTransform(AffineTransform(ltisys.getOutputFrame,r.getInputFrame,eye(length(ustar)),ustar));
r.getInputFrame.addTransform(AffineTransform(r.getInputFrame,ltisys.getOutputFrame,eye(length(ustar)),-ustar));

runDRCControl(r,ltisys,'mit_drc_robot','TRUE_ROBOT_STATE','ACTUATOR_CMDS',struct());
end
