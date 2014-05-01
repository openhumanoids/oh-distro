function drakeWalkingMomentum(use_mex,use_ik)

addpath(fullfile(getDrakePath,'examples','ZMP'));

%navgoal = [rand();randn();0;0;0;pi/2*randn()];
navgoal = [1;0;0;0;0;0];

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.002;
if (nargin>0); options.use_mex = use_mex;
else options.use_mex = true; end
if (nargin<2); use_ik = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp_zero_back.mat'));
xstar(1) = 0*randn();
xstar(2) = 0*randn();
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.05;

nq = getNumDOF(r);

x0 = xstar;

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
request.params.min_num_steps = 2;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.24;
request.params.max_step_width = 0.3;
request.params.nom_forward_step = 0.275;
request.params.max_forward_step = 0.4;
request.params.ignore_terrain = true;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_command = 0;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.75;
request.default_step_params.step_height = 0.05;
request.default_step_params.mu = 1.0;
request.default_step_params.constrain_full_foot_pose = true;
footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
walking_plan = walking_planner.plan_walking(r, request, true);
walking_ctrl_data = walking_planner.plan_walking(r, request, false);
walking_ctrl_data.supports = walking_ctrl_data.supports{1}; % TODO: fix this

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


% compute angular momentum trajectory from kinematic plan
% this would be replaced by dynamic plan
% qtraj = PPTrajectory(spline(ts,walking_plan.xtraj(1:nq,:)));
% qdtraj = fnder(qtraj,1);
% k = zeros(3,length(ts));
% comz = zeros(1,length(ts));
% for i=1:length(ts)
%   t=ts(i);
%   q=qtraj.eval(t);
%   qd=qdtraj.eval(t);
%   kinsol = doKinematics(r,q,false,true);
%   A = getCMM(r,kinsol);
%   k(:,i) = A(1:3,:)*qd;
%   com = getCOM(r,kinsol);
%   comz(i) = com(3);
% end
% ktraj = PPTrajectory(spline(ts,k));
%comztraj = PPTrajectory(spline(ts,comz));
%dcomztraj = fnder(comztraj,1);

ankle_ind = findJointIndices(r,'ak');

ctrl_data = SharedDataHandle(struct(...
  'is_time_varying',true,...
  'x0',[walking_ctrl_data.zmptraj.eval(T);0;0],...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',[walking_ctrl_data.supports{:}],...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'trans_drift',[0;0;0],...
  'qtraj',x0(1:nq),...
  'comtraj',walking_ctrl_data.comtraj,...
  'K',walking_ctrl_data.K,...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'neck')]));


% instantiate QP controller
options.dt = 0.002;
options.slack_limit = 100;
% options.w_qdd = 1.0*ones(nq,1);
% options.w_qdd(findJointIndices(r,'leg'))=0;
options.debug = false;
options.contact_threshold = 0.001;


if (use_ik)
	qp = MomentumControlBlock(r,{},ctrl_data,options);

	% feedback QP controller with atlas
	ins(1).system = 1;
	ins(1).input = 2;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(qp,r,[],[],ins,outs);
	clear ins outs;

	% feedback PD block 
% 	options.Kp = 270.0*ones(nq,1);
% 	options.Kd = 30.0*ones(nq,1);
% 	options.Kp(ankle_ind) = 80;
% 	options.Kd(ankle_ind) = 10;
	pd = WalkingPDBlock(r,ctrl_data,options);
	ins(1).system = 1;
	ins(1).input = 1;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(pd,sys,[],[],ins,outs);
	clear ins outs;

else
	lfoot_motion = FootMotionControlBlock(r,'l_foot',ctrl_data);
	rfoot_motion = FootMotionControlBlock(r,'r_foot',ctrl_data);
	pelvis_motion = TorsoMotionControlBlock(r,'pelvis',ctrl_data);
	torso_motion = TorsoMotionControlBlock(r,'utorso',ctrl_data);
	motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame,...
		pelvis_motion.getOutputFrame,torso_motion.getOutputFrame};
	qp = MomentumControlBlock(r,motion_frames,ctrl_data,options);

	% feedback QP controller with atlas
	ins(1).system = 1;
	ins(1).input = 2;
	ins(2).system = 1;
	ins(2).input = 3;
	ins(3).system = 1;
	ins(3).input = 4;
	ins(4).system = 1;
	ins(4).input = 5;
	ins(5).system = 1;
	ins(5).input = 6;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(qp,r,[],[],ins,outs);
	clear ins outs;

	% feedback PD block 
	pd = SimplePDBlock(r);
	ins(1).system = 1;
	ins(1).input = 1;
	ins(2).system = 2;
	ins(2).input = 2;
	ins(3).system = 2;
	ins(3).input = 3;
	ins(4).system = 2;
	ins(4).input = 4;
	ins(5).system = 2;
	ins(5).input = 5;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(pd,sys,[],[],ins,outs);
	clear ins outs;

	% feedback body motion control blocks
	ins(1).system = 2;
	ins(1).input = 1;
	ins(2).system = 2;
	ins(2).input = 3;
	ins(3).system = 2;
	ins(3).input = 4;
	ins(4).system = 2;
	ins(4).input = 5;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(lfoot_motion,sys,[],[],ins,outs);
	clear ins outs;

	ins(1).system = 2;
	ins(1).input = 1;
	ins(2).system = 2;
	ins(2).input = 3;
	ins(3).system = 2;
	ins(3).input = 4;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(rfoot_motion,sys,[],[],ins,outs);
	clear ins outs;

	ins(1).system = 2;
	ins(1).input = 1;
	ins(2).system = 2;
	ins(2).input = 3;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(pelvis_motion,sys,[],[],ins,outs);
	clear ins outs;

	ins(1).system = 2;
	ins(1).input = 1;
	outs(1).system = 2;
	outs(1).output = 1;
	sys = mimoFeedback(torso_motion,sys,[],[],ins,outs);
	clear ins outs;
end

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);
playback(v,traj,struct('slider',true));

if 1%plot_comtraj
  dt = 0.001;
  tts = 0:dt:T;
  xtraj_smooth=smoothts(traj.eval(tts),'e',150);
  dtraj = fnder(PPTrajectory(spline(tts,xtraj_smooth)));
  qddtraj = dtraj(nq+(1:nq));

  lfoot = findLinkInd(r,'l_foot');
  rfoot = findLinkInd(r,'r_foot');
  
  lstep_counter = 0;
  rstep_counter = 0;

  rms_zmp = 0;
  rms_com = 0;
  rms_foot = 0;
  
  for i=1:length(ts)
    xt=traj.eval(ts(i));
    q=xt(1:nq);
    qd=xt(nq+(1:nq));
    qdd=qddtraj.eval(ts(i));

    kinsol = doKinematics(r,q);

    [com(:,i),J]=getCOM(r,kinsol);
    Jdot = forwardJacDot(r,kinsol,0);
    comdes(:,i)=walking_ctrl_data.comtraj.eval(ts(i));
    zmpdes(:,i)=walking_ctrl_data.zmptraj.eval(ts(i));
    zmpact(:,i)=com(1:2,i) - com(3,i)/9.81 * (J(1:2,:)*qdd + Jdot(1:2,:)*qd);

    lfoot_cpos = contactPositions(r,kinsol,lfoot);
    rfoot_cpos = contactPositions(r,kinsol,rfoot);
    
    lfoot_p = forwardKin(r,kinsol,lfoot,[0;0;0],1);
    rfoot_p = forwardKin(r,kinsol,rfoot,[0;0;0],1);
    
		lfoot_pos(:,i) = lfoot_p;
		rfoot_pos(:,i) = lfoot_p;

		if any(lfoot_cpos(3,:) < 1e-4)
      lstep_counter=lstep_counter+1;
      lfoot_steps(:,lstep_counter) = lfoot_p;
		end
    if any(rfoot_cpos(3,:) < 1e-4)
      rstep_counter=rstep_counter+1;
      rfoot_steps(:,rstep_counter) = rfoot_p;
    end
    
    rfoottraj = walking_ctrl_data.link_constraints(1).traj;
    lfoottraj = walking_ctrl_data.link_constraints(2).traj;
    
    lfoot_des = eval(lfoottraj,ts(i));
    lfoot_des(3) = max(lfoot_des(3), 0.0811);     % hack to fix footstep planner bug
    rms_foot = rms_foot+norm(lfoot_des([1:3])-lfoot_p([1:3]))^2;
  
    rfoot_des = eval(rfoottraj,ts(i));
    rfoot_des(3) = max(rfoot_des(3), 0.0811);     % hack to fix footstep planner bug
    rms_foot = rms_foot+norm(rfoot_des([1:3])-rfoot_p([1:3]))^2;

    rms_zmp = rms_zmp+norm(zmpdes(:,i)-zmpact(:,i))^2;
    rms_com = rms_com+norm(comdes(:,i)-com(1:2,i))^2;
  end

  rms_zmp = sqrt(rms_zmp/length(ts))
  rms_com = sqrt(rms_com/length(ts))
  rms_foot = sqrt(rms_foot/(lstep_counter+rstep_counter))
  
  figure(2); 
  clf; 
  subplot(2,1,1);
  plot(ts,zmpdes(1,:),'b');
  hold on;  
  plot(ts,zmpact(1,:),'r.-');
  plot(ts,comdes(1,:),'g');
  plot(ts,com(1,:),'m.-');
  hold off;  
  
  subplot(2,1,2);
  plot(ts,zmpdes(2,:),'b');
  hold on;  
  plot(ts,zmpact(2,:),'r.-');
  plot(ts,comdes(2,:),'g');
  plot(ts,com(2,:),'m.-');
  hold off;  

  figure(3)
  clf;
  plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
  hold on;  
  plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);
  %plot(comdes(1,:),comdes(2,:),'g','LineWidth',3);
  %plot(com(1,:),com(2,:),'m.-','LineWidth',1);
 
  left_foot_steps = eval(lfoottraj,lfoottraj.getBreaks);
  for i=1:size(left_foot_steps,2);
    cpos = rpy2rotmat(left_foot_steps(4:6,i)) * getBodyContacts(r,lfoot) + repmat(left_foot_steps(1:3,i),1,4);
    if all(cpos(3,:)<=0.001)
      plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
      plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',2);
      plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
      plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
      plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
    end
  end
  
  right_foot_steps = eval(rfoottraj,rfoottraj.getBreaks);
  for i=1:size(right_foot_steps,2);
    cpos = rpy2rotmat(right_foot_steps(4:6,i)) * getBodyContacts(r,rfoot) + repmat(right_foot_steps(1:3,i),1,4);
    if all(cpos(3,:)<=0.001)
      plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
      plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
      plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
      plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
    end
  end
  
  for i=1:lstep_counter
    cpos = rpy2rotmat(lfoot_steps(4:6,i)) * getBodyContacts(r,lfoot) + repmat(lfoot_steps(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end

  for i=1:rstep_counter
    cpos = rpy2rotmat(rfoot_steps(4:6,i)) * getBodyContacts(r,rfoot) + repmat(rfoot_steps(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end

  plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
  plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);

  axis equal;
  
if rms_com > length(footstep_plan.footsteps)*0.5
  error('drakeWalking unit test failed: error is too large');
  navgoal
end


% for i=1:6
% 	figure;
% 	fnplt(walking_ctrl_data.link_constraints(2).traj(i))
% 	hold on;
% 	plot(ts,lfoot_pos(i,:));
% 	hold off;
% end

% keyboard;

end
