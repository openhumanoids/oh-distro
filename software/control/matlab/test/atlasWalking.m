function atlasWalking
%NOTEST

use_foot_pd = true;

addpath(fullfile(getDrakePath,'examples','ZMP'));

joint_str = {'leg'};% <---- force controlled joints, cell array of (sub)strings 

% load robot model
r = Atlas();
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
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
% qdes = xstar(1:nq);
% atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,5);

gains_copy = getAtlasGains();
% reset force gains 
gains.k_f_p(joint_act_ind) = gains_copy.k_f_p(joint_act_ind);
gains.ff_f_d(joint_act_ind) = gains_copy.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains_copy.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains_copy.ff_qd_d(joint_act_ind);
% set joint position gains to 0 
gains.k_q_p(joint_act_ind) = 0;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq);
q0 = x0(1:nq);

% create navgoal
R = rpy2rotmat([0;0;x0(6)]);
v = R*[1;0;0];
navgoal = [x0(1)+v(1);x0(2)+v(2);0;0;0;x0(6)];

% create footstep and ZMP trajectories
footstep_planner = StatelessFootstepPlanner();
request = drc.footstep_plan_request_t();
request.utime = 0;
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.goal_pos = encodePosition3d(navgoal);
request.num_goal_steps = 0;
request.num_existing_steps = 0;
request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 20;
request.params.min_num_steps = 1;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.28;
request.params.max_step_width = 0.32;
request.params.nom_forward_step = 0.18;
request.params.max_forward_step = 0.20;
request.params.nom_upward_step = 0.2;
request.params.nom_downward_step = 0.2;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_mode = drc.footstep_plan_params_t.HORIZONTAL_PLANE;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.15;
request.default_step_params.step_height = 0.065;
request.default_step_params.mu = 0.8;
request.default_step_params.constrain_full_foot_pose = true;
request.default_step_params.drake_min_hold_time = 1.25; %sec

footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
request.use_new_nominal_state = true;
request.new_nominal_state = r.getStateFrame().lcmcoder.encode(0, x0);
walking_plan = walking_planner.plan_walking(r, request, true);
lc = lcm.lcm.LCM.getSingleton();
lc.publish('FOOTSTEP_PLAN_RESPONSE', footstep_plan.toLCM());
% lc.publish('WALKING_TRAJ_RESPONSE', walking_plan.toLCM());
walking_ctrl_data = walking_planner.plan_walking(r, request, false);

% No-op: just make sure we can cleanly encode and decode the plan as LCM
walking_ctrl_data = WalkingControllerData.from_walking_plan_t(walking_ctrl_data.toLCM());

ts = walking_plan.ts;
T = ts(end);

% plot walking traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

for i=1:length(ts)
	lcmgl.glColor3f(0, 0, 1);
	lcmgl.sphere([walking_ctrl_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
  lcmgl.glColor3f(0, 1, 0);
	lcmgl.sphere([walking_ctrl_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
end
lcmgl.switchBuffers();

% compute smooth polynomial footstep trajectories
for i=1:length(walking_ctrl_data.link_constraints)
  traj = walking_ctrl_data.link_constraints(i).traj;
  breaks = unique(traj.getBreaks());
  points = traj.eval(breaks);
  zpoints = points(3, :);
  change_indices = [true diff(diff(zpoints)) ~= 0 true];
  
%   new_points = zeros(size(points));
%   for j = 1 : size(points, 1)
% %     new_points(j, :) = filtfilt(b, a, points(j, :));
%     new_points(j, :) = smooth(points(j, :), 3, 'moving');
%   end
  
  
%   rough_velocities = diff(zpoints) ./ diff(breaks);
%   similar_velocity_indices = [true abs(diff(rough_velocities)) < 0.01 true];
%   indices = change_indices & ~similar_velocity_indices;
  new_traj = PPTrajectory(pchip(breaks(change_indices), points(:, change_indices)));
%   ndiff = diff(points')';
%   new_traj = PPTrajectory(pchipDeriv(breaks,points,ndiff));
  walking_ctrl_data.link_constraints(i).traj = new_traj;
  walking_ctrl_data.link_constraints(i).dtraj = fnder(new_traj);
  walking_ctrl_data.link_constraints(i).ddtraj = fnder(new_traj,2);  
end


ctrl_data = QPControllerData(true,struct(...
  'acceleration_input_frame',AtlasCoordinates(r),...
  'D',-0.89/9.81*eye(2),... % assumed COM height
  'Qy',eye(2),...
  'S',walking_ctrl_data.S,... % always a constant
  's1',walking_ctrl_data.s1,...
  's2',walking_ctrl_data.s2,...
  's1dot',walking_ctrl_data.s1dot,...
  's2dot',walking_ctrl_data.s2dot,...
  'x0',[walking_ctrl_data.zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
	'qtraj',q0,...
  'comtraj',walking_ctrl_data.comtraj,...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',walking_ctrl_data.supports,...
  'mu',walking_ctrl_data.mu,...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'y0',walking_ctrl_data.zmptraj,...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'neck');findJointIndices(r,'back')]));

save('walking_ctrl_data.mat','walking_ctrl_data');

% instantiate QP controller
options.slack_limit = 30;
options.w_qdd = 0*ones(nq,1);
options.W_kdot = 0*eye(3);
options.w_grf = 0.0;
options.w_slack = 0.05;
options.Kp_accel = 0.0;
options.debug = false;
options.use_mex = true;
options.contact_threshold = 0.01;
options.output_qdd = true;
options.solver = 0; % 0 fastqp, 1 gurobi
options.input_foot_contacts = true;

if use_foot_pd 
  options.Kp = [20; 20; 20; 10; 20; 10];
  options.Kd = getDampingGain(options.Kp,0.6);
  lfoot_motion = FootMotionControlBlock(r,'l_foot',ctrl_data,options);
	rfoot_motion = FootMotionControlBlock(r,'r_foot',ctrl_data,options);

  options.Kp = 20*[0; 0; 1; 1; 1; 1];
  options.Kd = getDampingGain(options.Kp,0.4);
	pelvis_motion = PelvisMotionControlBlock(r,'pelvis',ctrl_data,options);
	motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame,...
    pelvis_motion.getOutputFrame};
  options.body_accel_input_weights = 0.25*[1 1 1];
	qp = QPController(r,motion_frames,ctrl_data,options);
else
  qp = QPController(r,{},ctrl_data,options);
end

vo = VelocityOutputIntegratorBlock(r,options);
options.use_lcm = true;
options.use_contact_logic_OR = true;
fcb = FootContactBlock(r,ctrl_data,options);
fshift = FootstepPlanShiftBlock(r,ctrl_data);

if use_foot_pd
  options.use_ik = false;
end

% cascade IK/PD block
options.Kp = 50.0*ones(nq,1);
options.Kd = 8*ones(nq,1);
pd = IKPDBlock(r,ctrl_data,options);

if use_foot_pd
  ins(1).system = 1;
  ins(1).input = 1;
  ins(2).system = 1;
  ins(2).input = 2;
  ins(3).system = 1;
  ins(3).input = 3;
  ins(4).system = 2;
  ins(4).input = 1;
  ins(5).system = 2;
  ins(5).input = 3;
  ins(6).system = 2;
  ins(6).input = 4;
  ins(7).system = 2;
  ins(7).input = 5;
  ins(8).system = 2;
  ins(8).input = 6;
else
  ins(1).system = 1;
  ins(1).input = 1;
  ins(2).system = 1;
  ins(2).input = 2;
  ins(3).system = 1;
  ins(3).input = 3;
  ins(4).system = 2;
  ins(4).input = 1;
  ins(5).system = 2;
  ins(5).input = 3;
end
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
qp_sys = mimoCascade(pd,qp,[],ins,outs);
clear ins;

toffset = -1;
tt=-1;

torque_fade_in = 0.1; % sec, to avoid jumps at the start

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

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

    q = x(1:nq);
    qd = x(nq+(1:nq));
  
    x = [q;qd];

    fc = output(fcb,tt,[],x);
  
    junk = mimoOutput(fshift,tt,[],x,fc);

    if use_foot_pd
      lfoot_ddot = output(lfoot_motion,tt,[],x);
      rfoot_ddot = output(rfoot_motion,tt,[],x);
      pelvis_ddot = output(pelvis_motion,tt,[],x);
      u_and_qdd = output(qp_sys,tt,[],[q0; x; fc; x; fc; lfoot_ddot; rfoot_ddot; pelvis_ddot]);
    else
      u_and_qdd = output(qp_sys,tt,[],[q0; x; fc; x; fc]);
    end
    
    u=u_and_qdd(1:nu);
    qdd=u_and_qdd(nu+(1:nq));

    qd_int_state = mimoUpdate(vo,tt,qd_int_state,x,qdd,fc);
    qd_ref = mimoOutput(vo,tt,qd_int_state,x,qdd,fc);

    % fade in desired torques to avoid spikes at the start
    udes(joint_act_ind) = u(joint_act_ind);
    tau = tau(act_idx_map);
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);

    qddes(joint_act_ind) = qd_ref(joint_act_ind);

    ref_frame.publish(t,[q0(act_idx_map);qddes;udes],'ATLAS_COMMAND');
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
