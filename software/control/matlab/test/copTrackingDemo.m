function copTrackingDemo
%NOTEST
addpath(fullfile(getDrakePath,'examples','ZMP'));

joint_str = {'leg'};% <---- cell array of (sub)strings  

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

act_idx_map = getActuatedJoints(r);
gains = getAtlasGains(input_frame); % change gains in this file

joint_ind = [];
joint_act_ind = [];
for i=1:length(joint_str)
  joint_ind = union(joint_ind,find(~cellfun(@isempty,strfind(state_plus_effort_frame.coordinates(1:nq),joint_str{i}))));
  joint_act_ind = union(joint_act_ind,find(~cellfun(@isempty,strfind(input_frame.coordinates,joint_str{i}))));
end

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,5);

gains2 = getAtlasGains(input_frame); 
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains2.k_f_p(joint_act_ind); 
gains.ff_f_d(joint_act_ind) = gains2.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains2.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains2.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = gains.k_q_p(joint_act_ind)*0.0;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq); 
q0 = x0(1:nq); 

% create figure 8 zmp traj
T = 20;
dt = 0.01;
ts = 0:0.01:T;
nt = T/dt;
radius = 0.05; % 8 loop radius
zmpx = [radius*sin(4*pi/T * ts(1:nt/2)), radius*sin(4*pi/T * ts(1:nt/2+1))];
zmpy = [radius-radius*cos(4*pi/T * ts(1:nt/2)), -radius+radius*cos(4*pi/T * ts(1:nt/2+1))];
zmpknots = [zmpx;zmpy;0*zmpx];

% create navgoal
R = rpy2rotmat([0;0;x0(6)]);
zmpknots = R*zmpknots;
zmptraj = PPTrajectory(spline(ts,zmpknots(1:2,:)));

rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');
foot_pos = contactPositions(r,q0, [rfoot_ind, lfoot_ind]);
foot_center = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
zmptraj = zmptraj + foot_center;
zmptraj = zmptraj.setOutputFrame(desiredZMP);

% plot walking traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'zmp-traj');

for i=1:length(ts)
  lcmgl.glColor3f(0, 1, 0);
	lcmgl.sphere([zmptraj.eval(ts(i));0], 0.01, 20, 20);  
end
lcmgl.switchBuffers();

% set up QP controller params
foot_support = SupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));

kinsol = doKinematics(r,q0,false,true);
com = getCOM(r,kinsol);
options.com0 = com(1:2);
K = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3),zmptraj,options);

ctrl_data = SharedDataHandle(struct(...
  'is_time_varying',true,...
  'x0',[zmptraj.eval(T);0;0],...
  'support_times',0,...
  'supports',foot_support,...
  'ignore_terrain',false,...
  'trans_drift',[0;0;0],...
  'qtraj',q0,...
  'K',K,...
  'mu',1,...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));

% instantiate QP controller
options.slack_limit = 20;
options.w = 0.1;
options.lcm_foot_contacts = false;
options.debug = false;
options.use_mex = true;
options.contact_threshold = 0.05;
options.output_qdd = true;

qp = MomentumControlBlock(r,{},ctrl_data,options);

% cascade PD block
options.Kp = 25.0*ones(nq,1);
options.Kp = 10.0*ones(nq,1);
pd = SimplePDBlock(r,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 1;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
sys = mimoCascade(pd,qp,[],ins,outs);
clear ins;

qddes = zeros(nu,1);
udes = zeros(nu,1);

toffset = -1;
tt=-1;
dt = 0.001;

process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

torque_fade_in = 0.75; % sec, to avoid jumps at the start

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

xtraj = [];

qd_int = 0;
eta = 0.9;
while tt<T
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;

    tau = x(2*nq+(1:nq));
    tau = tau(act_idx_map);
    
    % get estimated state
    kf_state = kf.update(tt,kf_state,x(1:nq));
    x = kf.output(tt,kf_state,x(1:nq));

    xtraj = [xtraj x];
    q = x(1:nq);
    qd = x(nq+(1:nq));
  
    u_and_qdd = output(sys,tt,[],[q0;q;qd;q;qd]);
    u=u_and_qdd(1:nu);
    qdd=u_and_qdd(nu+1:end);
    udes(joint_act_ind) = u(joint_act_ind);
    
    % fade in desired torques to avoid spikes at the start
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);
    
    % compute desired velocity
    qd_int = qd_int + eta*qdd*dt;
    qddes_state_frame = qd_int;
    qddes_input_frame = qddes_state_frame(act_idx_map);
    qddes(joint_act_ind) = qddes_input_frame(joint_act_ind);
    
    ref_frame.publish(t,[q0(act_idx_map);qddes;udes],'ATLAS_COMMAND');
  end
end

disp('moving back to fixed point using position control.');
gains = getAtlasGains(input_frame); % change gains in this file
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,6);


% plot tracking performance
alpha = 0.05;
zmpact = [];
for i=1:size(xtraj,2)
  x = xtraj(:,i);
  q = x(1:nq);
  qd = x(nq+(1:nq));  
  
  if i==1
		qdd = 0*qd;
	else
		qdd = (1-alpha)*qdd_prev + alpha*(qd-qd_prev)/dt;
  end
  qd_prev = qd;
	qdd_prev = qdd;  

  kinsol = doKinematics(r,q,false,true);
  [com,J] = getCOM(r,kinsol);
	J = J(1:2,:); 
	Jdot = forwardJacDot(r,kinsol,0);
  Jdot = Jdot(1:2,:);
	
	% hardcoding D for ZMP output dynamics
	D = -1.04./9.81*eye(2); 

	comdd = Jdot * qd + J * qdd;
	zmp = com(1:2) + D * comdd;
	zmpact = [zmpact zmp];
end

zmpknots = zmptraj.eval(zmptraj.getBreaks());
figure(11);
plot(zmpact(1,:),zmpact(2,:),'r');
hold on;
plot(zmpknots(1,:),zmpknots(2,:),'g');
hold off;

end
