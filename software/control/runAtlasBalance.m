function runAtlasBalance(xD,uD)
% xD,uD is the desired state and control fixed point
options.floating = true;
options.view = 'right';
m = PlanarRigidBodyModel('../../ros_workspace/atlas_description/urdf/atlas_robot.urdf',options);
r = TimeSteppingRigidBodyManipulator(m,.001);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RKneePitch = 0.1;
x0.LKneePitch = 0.1;
x0.LAnklePitch = -0.1;
x0.RAnklePitch = -0.1;
x0.base_link_z = 1;
u0 = Point(r.getInputFrame);
if(nargin<1)
    [xD,uD] = computeInitialPosture(double(x0),double(u0),r);
end
num_q = r.getNumStates/2;
num_u = r.getNumInputs;
k_D = [-5*[zeros(num_u,num_q-num_u) eye(num_u)] -5*[zeros(num_u,num_q-num_u) eye(num_u)]];
k_y0 = -k_D*xD+uD;
control = AffineSystem([],[],[],[],[],[],[],k_D,k_y0);
control = control.setInputFrame(r.getOutputFrame);
control = control.setOutputFrame(r.getInputFrame);
r_cl = feedback(r,control);
%r_cl = cascade(feedback(r,control),v);
%simulate(r_cl,[0 1],x0);
x0 = Point(r.getStateFrame,xD);
x0.RShoulderPitch = x0.RShoulderPitch;
x0.LShoulderPitch = x0.LShoulderPitch;
x0.BackPitch = x0.BackPitch+0.1;
xtraj = simulate(r_cl,[0 2],xD);
save PDcontrol.mat v xtraj;
v.playback_speed = 0.2;
v.playback(xtraj);

keyboard;
end

function [xD,uD] = computeInitialPosture(x0,u0,obj)
problem.objective = @(x) 0;
problem.x0 = [x0;u0];
problem.nonlcon = @(xu) mycon(xu,obj);
problem.solver = 'fmincon';
problem.options=optimset('Algorithm','interior-point','Display','off');
[xu_sol,~,exitflag] = fmincon(problem);
success=(exitflag==1);
nX = obj.getNumStates();
nU = obj.getNumInputs();
xD = xu_sol(1:nX);
uD = xu_sol(nX+(1:nU));
if (~success)
    error('Drake:PlanarRigidBodyManipulator:ResolveConstraintsFailed','failed to resolve constraints');
end
end

function [c,ceq] = mycon(xu,obj)
manipObj = obj.manip;
nX = manipObj.getNumStates();
nq = nX/2;
nU = manipObj.getNumInputs();
RfootGroundContactInd = [187 189];
LfootGroundContactInd = [171 173];
footContactInd = false(190,1);
footContactInd([LfootGroundContactInd RfootGroundContactInd]) = true(4,1);
x = xu(1:nX);
u = xu(nX+(1:nU));
q = x(1:nq);
contactCandidate = contactConstraints(manipObj,q);
c = -[jointLimits(manipObj,q);contactCandidate(~footContactInd)];
ceq = [stateConstraints(manipObj,x);x-update(obj,0,x,u);contactCandidate(footContactInd)];
end