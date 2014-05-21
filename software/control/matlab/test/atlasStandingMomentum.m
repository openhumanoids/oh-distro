function atlasStandingMomentum
%NOTEST

error('Scott has to fix this script');

% test function for standing atlas using the momentum-based QP controller. 
% using the joint_str variable, the user can select a subset of leg joints with which
% to do torque control.
% useful for tuning gains, testing basic balancing capabilities, or making
% atlas dance.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT/MOVEMENT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint_str = {'leg'};% <---- cell array of (sub)strings  

% INPUT SIGNAL PARAMS %%%%%%%%%%%%%
dim = 3; % what dimension to move COM: x/y/z (1/2/3)
T = 15;% <--- signal duration (sec)

% chirp params
amp = 0.02;% <---- meters, COM DELTA
chirp_f0 = 0.2;% <--- chirp starting frequency
chirp_fT = 0.2;% <--- chirp ending frequency
chirp_sign = -1;% <--- -1: negative, 1: positive, 0: centered about offset 

% random pose params for sys id tests
use_random_traj = false; % if true, ignores chirp params
num_random_pose = 5;
pose_hold_time = 5; % sec
pose_move_time = 10; % sec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if use_random_traj
  T = (pose_hold_time+pose_move_time)*num_random_pose;
  r_ch = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'),0.003);
else
  ts = linspace(0,T,800);
end

% load robot model
r = Atlas();
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

act_idx_map = getActuatedJoints(r);
gains = getAtlasGains(); % change gains in this file

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

gains_copy = getAtlasGains(); 
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains_copy.k_f_p(joint_act_ind); 
gains.ff_f_d(joint_act_ind) = gains_copy.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains_copy.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains_copy.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = 0;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq); 
q0 = x0(1:nq);
com0 = getCOM(r,q0);

rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');

% set up QP controller params
foot_support = SupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));

if use_random_traj
  [xposes,info,~,~,~,constraints,ikoptions] = randomPose(r_ch,x0,num_random_pose);
  [traj,info] = interpolatingTraj(r_ch,[0 T],xposes,x0,pose_hold_time*num_random_pose/T,constraints,ikoptions);
  traj = traj.setOutputFrame(r.getStateFrame);
  qtraj = traj(1:nq);

  ts = 0:0.01:traj.tspan(end);
  comztraj = zeros(1,length(ts));
  
  for i=1:length(ts)
    kinsol = doKinematics(r,qtraj.eval(ts(i)));
    com = getCOM(r,kinsol);
    comztraj(i) = com(3);
  end
  comz_traj = PPTrajectory(foh(ts,comztraj));
  fnplt(comz_traj)
  
  dcomz_traj= ConstantTrajectory(0);
  ddcomz_traj= ConstantTrajectory(0);
  
  % build TI-ZMP controller
  foot_pos = contactPositions(r,q0,false,struct('terrain_only',true,'body_idx',[rfoot_ind, lfoot_ind])); 
  ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
  comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';%mean(foot_pos(1:2,ch(1:end-1)),2);
  
  limp = LinearInvertedPendulum(com0(3));
  K = lqr(limp,comgoal);
  
  ctrl_data = SharedDataHandle(struct(...
    'is_time_varying',false,...
    'x0',[comgoal;0;0],...
    'support_times',0,...
    'supports',foot_support,...
    'ignore_terrain',false,...
    'trans_drift',[0;0;0],...
    'qtraj',qtraj,...
    'K',K,...
    'comz_traj',comz_traj,...
    'dcomz_traj',dcomz_traj,...
    'ddcomz_traj',ddcomz_traj,...
    'mu',1,...
    'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));
  
else
  comtraj = ConstantTrajectory(com0);
  
  input_traj = chirpTraj(amp,chirp_f0,chirp_fT,T,0,chirp_sign);
  fade_window = 2; % sec
  fader = PPTrajectory(foh([0 fade_window T-fade_window T],[0 1 1 0]));
  input_traj = fader*input_traj;
  
  if dim==1
    traj_in_robot_frame = [input_traj;0;0];
  elseif dim==2
    traj_in_robot_frame = [0;input_traj;0];
  else
    traj_in_robot_frame = [0;0;input_traj];
  end
  
  R = rpy2rotmat([0;0;x0(6)]);
  comtraj = comtraj + R*traj_in_robot_frame;
  
  comz_traj = comtraj(3);
  dcomz_traj= fnder(comz_traj,1);
  ddcomz_traj= fnder(dcomz_traj,1);
  
  % get foot positions
  kinsol = doKinematics(r,q0);
  rfoot0 = forwardKin(r,kinsol,rfoot_ind,[0;0;0],1);
  lfoot0 = forwardKin(r,kinsol,lfoot_ind,[0;0;0],1);
  
  cost = Point(r.getStateFrame,1);
  cost.base_x = 0;
  cost.base_y = 0;
  cost.base_z = 0;
  cost.base_roll = 1000;
  cost.base_pitch = 1000;
  cost.base_yaw = 1000;
  cost.back_bkz = 10;
  cost.back_bky = 100;
  cost.back_bkx = 100;
  cost.r_arm_usy = 10;
  cost.r_arm_shx = 10;
  cost.r_arm_ely = 10;
  cost.r_arm_elx = 10;
  cost.r_arm_uwy = 10;
  cost.r_arm_mwx = 10;
  cost.l_arm_usy = 10;
  cost.l_arm_shx = 10;
  cost.l_arm_ely = 10;
  cost.l_arm_elx = 10;
  cost.l_arm_uwy = 10;
  cost.l_arm_mwx = 10;
  
  cost = double(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setQ(diag(cost(1:nq)));
  
  for i=1:length(ts)
    t = ts(i);
    if (i>1)
      
      kc_com = constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,true,r,comtraj.eval(t),comtraj.eval(t));
      rfarg = {constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,rfoot_ind,[0;0;0],rfoot0(1:3),rfoot0(1:3)),...
        constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,rfoot_ind,rfoot0(4:end),rfoot0(4:end))};
      lfarg = {constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,r,lfoot_ind,[0;0;0],lfoot0(1:3),lfoot0(1:3)),...
        constructRigidBodyConstraint(RigidBodyConstraint.WorldEulerConstraintType,true,r,lfoot_ind,lfoot0(4:end),lfoot0(4:end))};
      q(:,i) = inverseKin(r,q(:,i-1),q0,kc_com,rfarg{:},lfarg{:},ikoptions);
    else
      q = q0;
    end
  end
  
  qtraj = PPTrajectory(spline(ts,q));
  
  zmptraj = comtraj(1:2,:);
  zmptraj = zmptraj.setOutputFrame(desiredZMP);
    
  % plot zmp traj in drake viewer
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'zmp-traj');

  ts = 0:0.1:T;
  for i=1:length(ts)
    lcmgl.glColor3f(0, 1, 0);
    lcmgl.sphere([zmptraj.eval(ts(i));0], 0.01, 20, 20);
  end
  lcmgl.switchBuffers();


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
    'qtraj',qtraj,...
    'K',K,...
    'mu',1,...
    'comz_traj',comz_traj,...
    'dcomz_traj',dcomz_traj,...
    'ddcomz_traj',ddcomz_traj,...
    'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));
end

if 1 % visualize trajectory
  traj = [qtraj;0*qtraj];
  traj = traj.setOutputFrame(r.getStateFrame);  
  v = r.constructVisualizer;
  playback(v,traj,struct('slider',true));
end

% instantiate QP controller
options.slack_limit = 50;
options.w_qdd = 0.01*ones(nq,1);
options.W_hdot = diag([0;0;0;100;100;100]);
options.input_foot_contacts = false;
options.debug = false;
options.use_mex = true;
options.contact_threshold = 0.01;
options.output_qdd = true;

qp = MomentumControlBlock(r,{},ctrl_data,options);
vo = VelocityOutputIntegratorBlock(r,options);
fcb = FootContactBlock(r);

% cascade PD block
options.Kp = 50.0*ones(nq,1);
options.Kd = 8.0*ones(nq,1);
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

toffset = -1;
tt=-1;

torque_fade_in = 0.75; % sec, to avoid jumps at the start

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

% low pass filter for floating base velocities
alpha_v = 0.2;
float_v = 0;

udes = zeros(nu,1);
qddes = zeros(nu,1);
qd_int_state = zeros(nq+4,1);
while tt<T
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    tau = x(2*nq+(1:nq));
    
    % low pass filter floating base velocities
    float_v = (1-alpha_v)*float_v + alpha_v*x(nq+(1:6));
    x(nq+(1:6)) = float_v;
    
    q = x(1:nq);
    qd = x(nq+(1:nq));
    qt = fasteval(qtraj,tt);
 
    % get desired configuration
    qdes = qt(act_idx_map);    
    u_and_qdd = output(sys,tt,[],[qt;q;qd;q;qd]);
    u=u_and_qdd(1:nu);
    qdd=u_and_qdd(nu+(1:nq));
    
    fc = output(fcb,tt,[],[q;qd]);
    qd_int_state = mimoUpdate(vo,tt,qd_int_state,[q;qd],qdd,fc);
    qd_ref = mimoOutput(vo,tt,qd_int_state,[q;qd],qdd,fc);
    
    % fade in desired torques to avoid spikes at the start
    udes(joint_act_ind) = u(joint_act_ind);
    tau = tau(act_idx_map);
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);
    
    qddes(joint_act_ind) = qd_ref(joint_act_ind);
 
    ref_frame.publish(t,[qdes;qddes;udes],'ATLAS_COMMAND');
  end
end

disp('moving back to fixed point using position control.');
gains = getAtlasGains(); 
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,5);

end
