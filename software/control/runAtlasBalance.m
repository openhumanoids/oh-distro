function runAtlasBalance(xD,uD)
% xD,uD is the desired state and control fixed point
options.floating = true;
options.view = 'right';
m = PlanarRigidBodyModel('../../ros_workspace/deprecated/atlas_description/urdf/atlas_robot.urdf',options);
r = TimeSteppingRigidBodyManipulator(m,.0005);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
% x0.RHipPitch = -0.2;
% x0.LHipPitch = -0.2;
% x0.RKneePitch = 0.1;
% x0.LKneePitch = 0.1;
% x0.LAnklePitch = -0.1;
% x0.RAnklePitch = -0.1;
% x0.base_link_z = 1;
u0 = Point(r.getInputFrame);
% x0 = xD;
% u0 = uD;
if(nargin<3)
    [xD,uD] = computeInitialPostureSNOPT(double(x0),double(u0),r);
    xD = Point(r.getStateFrame,xD);
    uD = Point(r.getInputFrame,uD);
end
% [xD,uD] = computeInitialPosture(double(x0),double(u0),r);
[A,B] = linDynamics(r,double(xD),double(uD));
num_q = r.getNumStates/2;
num_u = r.getNumInputs;
% k_D = [zeros(num_u,num_q-num_u) diag([-10 -1 -1 -1 -1 -1 -6 -6 -8 -6 -6 -8]) ...
%     zeros(num_u,num_q-num_u) diag([-8 -2 -2 -2 -2 -2 -4 -4 -8 -4 -4 -8])];
% %k_D = [-5*[zeros(num_u,num_q-num_u) eye(num_u)] -5*[zeros(num_u,num_q-num_u) eye(num_u)]];
% k_y0 = -k_D*xD+uD;
% control = AffineSystem([],[],[],[],[],[],[],k_D,k_y0);
Q = diag([1 1 10 100 1 1 1 1 1 100 100 100 100 100 100 1 1 10 100 1 1 1 1 1 100 100 100 100 100 100]);
R = eye(num_u);
N = zeros(2*num_q,num_u);
[K,S] = dlqr(A,B,Q,R,N);
k_y0 = K*double(xD)+double(uD);
control = AffineSystem([],[],[],[],[],[],[],-K,k_y0);
control = control.setInputFrame(r.getOutputFrame);
control = control.setOutputFrame(r.getInputFrame);
r_cl = feedback(r,control);
%r_cl = cascade(feedback(r,control),v);
%simulate(r_cl,[0 1],x0);
x0 = xD;
x0.RShoulderPitch = x0.RShoulderPitch;
x0.LShoulderPitch = x0.LShoulderPitch;
x0.BackPitch = x0.BackPitch;
xtraj = simulate(r_cl,[0 2],x0);
tBreaks = xtraj.getBreaks();
utraj = control.output(tBreaks,[],xtraj);
save LQRcontrol.mat v xtraj utraj;
v.playback_speed = 0.2;
v.playback(xtraj);

keyboard;
end

function [xD,uD] = computeInitialPosture(x0,u0,obj)
problem.objective = @(x) 0;
problem.x0 = [x0;u0];
problem.nonlcon = @(xu) mycon(xu,obj);
problem.solver = 'fmincon';
problem.options=optimset('Algorithm','interior-point','Display','off','TolX',1e-6);
problem.lb = [obj.manip.joint_limit_min;-inf(obj.manip.num_q,1);obj.manip.umin];
problem.ub = [obj.manip.joint_limit_max;inf(obj.manip.num_q,1);obj.manip.umax];
% problem.ub(10) = -0.1;
% problem.lb(11) = 0.1;
% problem.ub(13) = -0.1;
% problem.lb(14) = 0.1;
problem.ub(12) =-0.1;
problem.ub(15) = -0.1;
[xu_sol,~,exitflag] = fmincon(problem);
%success=(exitflag==1);
nX = obj.getNumStates();
nU = obj.getNumInputs();
xD = xu_sol(1:nX);
uD = xu_sol(nX+(1:nU));
% if (~success)
%     error('Drake:PlanarRigidBodyManipulator:ResolveConstraintsFailed','failed to resolve constraints');
% end
% check the solution by myself
if(exitflag==1)
    success = 1;
else
    if(any(xu_sol-problem.lb<0)||any(xu_sol-problem.ub>0))
        success = 0;
    else
        [c,ceq] = problem.nonlcon(xu_sol);
        if(any(c>0))
            success = 0;
        elseif(max(abs(ceq)>1e-2))
            success = 0;
        else
            success = 1;
        end
    end
end
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
c = -contactCandidate(~footContactInd);
ceq = [stateConstraints(manipObj,x);...
    x-update(obj,0,x,u);...
    contactCandidate(footContactInd)];
end
function [xD,uD] = computeInitialPostureSNOPT(x0,u0,obj)
global SNOPT_USERFUN_WOgradient;
nX = length(x0);
nU = length(u0);
w0 = [x0;u0];
wlow = [obj.manip.joint_limit_min;-inf(obj.manip.num_q,1);obj.manip.umin];
whigh = [obj.manip.joint_limit_max;inf(obj.manip.num_q,1);obj.manip.umax];
Flow = zeros(1+2+2+nX,1);
Fhigh = zeros(1+2+2+nX,1);
snseti('Derivative option',0);
snprint('print.out');
SNOPT_USERFUN_WOgradient = @(w) fixedPointSnoptUserFun(obj,w);
[w,F,info] = snopt(w0,wlow,whigh,Flow,Fhigh,'snopt_userfun_wo_gradient',0,1);
if(info~=1)
    [str,cat] = snoptInfo(info);
    error('SNOPT:InfoNotOne',['SNOPT exited w/ info = ',num2str(info),'.\n',cat,': ',str,'\n  Check p19 of Gill06 for more information.']);  
end
xD = w(1:(nX/2));
uD = w((nX/2)+1:end);
end

function f = fixedPointSnoptUserFun(obj,xu)
manipObj = obj.manip;
nX = manipObj.getNumStates();
nq = nX/2;
nU = manipObj.getNumInputs();
% RfootGroundContactInd = [187 189];
% LfootGroundContactInd = [171 173];
% footContactInd = false(190,1);
% footContactInd([LfootGroundContactInd RfootGroundContactInd]) = true(4,1);
x = xu(1:nX);
u = xu(nX+(1:nU));
q = x(1:nq);
LFoot = manipObj.model.body(13);
RFoot = manipObj.model.body(16);
doKinematics(manipObj.model,q);
LFootContact = forwardKin(LFoot,LFoot.contact_pts(:,[5 7]));
RFootContact = forwardKin(RFoot,RFoot.contact_pts(:,[5 7]));
f = [0;LFootContact(2,:)';RFootContact(2,:)';x-update(obj,0,x,u)];
end

