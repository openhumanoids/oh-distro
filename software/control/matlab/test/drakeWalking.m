function drakeWalking(use_mex,use_bullet)

addpath(fullfile(getDrakePath,'examples','ZMP'));

plot_comtraj = false;
%navgoal = [0.5*randn();0.5*randn();0;0;0;pi/2*randn()];
navgoal = [1;0;0;0;0;0];

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.002;
if (nargin>0); options.use_mex = use_mex;
else options.use_mex = true; end
if (nargin<2)
  use_bullet = false; % test walking with the controller computing pairwise contacts using bullet
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 0*randn();
xstar(2) = 0*randn();
r = r.setInitialState(xstar);

if use_bullet
  r_bullet = RigidBodyManipulator();
  r_bullet = addRobotFromURDF(r_bullet,strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),[0;0;0],[0;0;0],options);
  r_bullet = addRobotFromURDF(r_bullet,strcat(fullfile(getDrakePath,'systems','plants','test'),'/ground_plane.urdf'),[xstar(1);xstar(2);0],zeros(3,1),struct('floating',false));
  r_bullet = TimeSteppingRigidBodyManipulator(r_bullet,options.dt,options);
  r_bullet = removeCollisionGroupsExcept(r_bullet,{'heel','toe'});
  r_bullet = compile(r_bullet);
end

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
request.params.max_num_steps = 30;
request.params.min_num_steps = 2;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.26;
request.params.max_step_width = 0.39;
request.params.nom_forward_step = 0.2;
request.params.max_forward_step = 0.45;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_mode = drc.footstep_plan_params_t.HORIZONTAL_PLANE;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.5;
request.default_step_params.drake_min_hold_time = 2.0;
request.default_step_params.step_height = 0.05;
request.default_step_params.mu = 1.0;
request.default_step_params.constrain_full_foot_pose = false;

footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
walking_planner.plan_walking(r, request, true);
walking_ctrl_data = walking_planner.plan_walking(r, request, false);

if use_bullet
  for i=1:length(walking_ctrl_data.supports)
    walking_ctrl_data.supports{1}(i)=walking_ctrl_data.supports{1}(i).setContactSurfaces(-ones(length(walking_ctrl_data.supports{1}(i).bodies),1));
  end
end

ts = 0:0.1:walking_ctrl_data.zmptraj.tspan(end);
T = ts(end);

ctrl_data = SharedDataHandle(struct(...
  'A',[zeros(2),eye(2); zeros(2,4)],...
  'B',[zeros(2); eye(2)],...
  'C',[eye(2),zeros(2)],...
  'Qy',eye(2),...
  'R',zeros(2),...
  'is_time_varying',true,...
  'S',walking_ctrl_data.S.eval(0),... % always a constant
  's1',walking_ctrl_data.s1,...
  's2',walking_ctrl_data.s2,...
  's1dot',walking_ctrl_data.s1dot,...
  's2dot',walking_ctrl_data.s2dot,...
  'x0',[walking_ctrl_data.zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
	'qtraj',x0(1:nq),...
	'comtraj',walking_ctrl_data.comtraj,...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',walking_ctrl_data.supports,...
  't_offset',0,...
  'mu',walking_ctrl_data.mu,...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'y0',walking_ctrl_data.zmptraj));

% instantiate QP controller
options.dt = 0.003;
options.slack_limit = 30.0;
options.w = 0.001;
options.lcm_foot_contacts = false;
options.debug = false;

if use_bullet
  options.multi_robot = r_bullet;
end
qp = QPControlBlock(r,ctrl_data,options);

% cascade footstep plan shift block
fs = FootstepPlanShiftBlock(r,ctrl_data,options);
sys = cascade(r,fs);

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,sys,[],[],ins,outs);
clear ins outs;

% feedback PD block
pd = WalkingPDBlock(r,ctrl_data);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

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

if plot_comtraj
  dt = 0.001;
  tts = 0:dt:T;
  qdd = zeros(nq,T/dt);
  xtraj_smooth=smoothts(eval(traj,tts),'e',150);
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

    if any(lfoot_cpos(3,:) < 1e-4)
      lstep_counter=lstep_counter+1;
      lfoot_pos(:,lstep_counter) = lfoot_p;
    end
    if any(rfoot_cpos(3,:) < 1e-4)
      rstep_counter=rstep_counter+1;
      rfoot_pos(:,rstep_counter) = rfoot_p;
    end

    lfoot_des = eval(foottraj.left.orig,ts(i));
    lfoot_des(3) = max(lfoot_des(3), 0.0811);     % hack to fix footstep planner bug
    rms_foot = rms_foot+norm(lfoot_des([1:3])-lfoot_p([1:3]))^2;

    rfoot_des = eval(foottraj.right.orig,ts(i));
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

  left_foot_steps = eval(foottraj.left.orig,foottraj.left.orig.getBreaks);
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

  right_foot_steps = eval(foottraj.right.orig,foottraj.right.orig.getBreaks);
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
    cpos = rpy2rotmat(lfoot_pos(4:6,i)) * getBodyContacts(r,lfoot) + repmat(lfoot_pos(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end

  for i=1:rstep_counter
    cpos = rpy2rotmat(rfoot_pos(4:6,i)) * getBodyContacts(r,rfoot) + repmat(rfoot_pos(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end

  plot(zmpdes(1,:),zmpdes(2,:),'b','LineWidth',3);
  plot(zmpact(1,:),zmpact(2,:),'r.-','LineWidth',1);

  axis equal;

if rms_com > length(footsteps)*0.5
  error('drakeWalking unit test failed: error is too large');
  navgoal
end

end
