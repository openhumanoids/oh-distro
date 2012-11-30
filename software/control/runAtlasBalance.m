function runAtlasBalance()
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
% Q = diag([750*ones(1,nq) 1*ones(1,nq)]);
% R = 0.001*eye(nu);

load('/home/scottk/Atlas/data/drc_fp.mat');

% [A,B,C,D,xn0] = dlinearize(r,dt,0,xstar,ustar);
% [K,S] = dlqr(A,B,Q,R);
c = ConstantSystem(r,length(xstar),ustar);

% ltisys = LinearSystem([],[],[],[],[],-K);
% 
% ltisys = setInputFrame(ltisys,CoordinateFrame([r.getStateFrame.name,' - ', mat2str(xstar,3)],length(xstar),r.getStateFrame.prefix));
% r.getStateFrame.addTransform(AffineTransform(r.getStateFrame,ltisys.getInputFrame,eye(length(xstar)),-xstar));
% ltisys.getInputFrame.addTransform(AffineTransform(ltisys.getInputFrame,r.getStateFrame,eye(length(xstar)),+xstar));
% 
% ltisys = setOutputFrame(ltisys,CoordinateFrame([r.getInputFrame.name,' + ',mat2str(ustar,3)],length(ustar),r.getInputFrame.prefix));
% ltisys.getOutputFrame.addTransform(AffineTransform(ltisys.getOutputFrame,r.getInputFrame,eye(length(ustar)),ustar));
% r.getInputFrame.addTransform(AffineTransform(r.getInputFrame,ltisys.getOutputFrame,eye(length(ustar)),-ustar));

runDRCControl(r,c,'mit_drc_robot','TRUE_ROBOT_STATE','ACTUATOR_CMDS',struct());

end

