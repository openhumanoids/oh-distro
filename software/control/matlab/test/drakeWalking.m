function drakeWalking(use_mex,use_ik,use_bullet,use_angular_momentum,random_navgoal)
%NOTEST
addpath(fullfile(getDrakePath,'examples','ZMP'));

plot_comtraj = true;

if (nargin<1); use_mex = true; end
if (nargin<2); use_ik = false; end
if (nargin<3); use_bullet = false; end
if (nargin<4); use_angular_momentum = false; end
if (nargin<5); random_navgoal = false; end

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_v4_fp.mat'));
if random_navgoal
  xstar(1) = randn();
  xstar(2) = randn();
  xstar(6) = pi*randn();
  navgoal = [xstar(1)+rand();xstar(2)+randn();0;0;0;pi*randn()];
else
  navgoal = [1;0;0;0;0;0]; % straight forward 1m
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.001;
options.atlas_version = 4;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/atlas_v4/model_minimal_contact.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

xstar(3) = xstar(3) + 0.007; % TODO REMOVE THIS ADJUSTMENT WHEN FOOT CONTACT POINT LOCATIONS ARE FIXED
% set initial state to fixed point
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(r);

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
request.params.nom_step_width = 0.24;
request.params.max_step_width = 0.3;
request.params.nom_forward_step = 0.5;
request.params.max_forward_step = 0.5;
request.params.nom_upward_step = 0.25;
request.params.nom_downward_step = 0.25;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_mode = drc.footstep_plan_params_t.HORIZONTAL_PLANE;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.4;
request.default_step_params.drake_min_hold_time = 0.75;
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

% No-op: just make sure we can cleanly encode and decode the plan as LCM
tic;
walking_ctrl_data = WalkingControllerData.from_walking_plan_t(walking_ctrl_data.toLCM());
fprintf(1, 'control data lcm code/decode time: %f\n', toc);

ts = walking_plan.ts;
T = ts(end);

if plot_comtraj
  % plot walking traj in drake viewer
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

  for i=1:length(ts)
    lcmgl.glColor3f(0, 0, 1);
    lcmgl.sphere([walking_ctrl_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
    lcmgl.glColor3f(0, 1, 0);
    lcmgl.sphere([walking_ctrl_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
  end
  lcmgl.switchBuffers();
end

traj = simulateWalking(r, walking_ctrl_data, ts, use_mex, use_ik, use_bullet, use_angular_momentum, true);

playback(v,traj,struct('slider',true));

if plot_comtraj
  dt = 0.001;
  tts = 0:dt:T;
  xtraj_smooth=smoothts(traj.eval(tts),'e',150);
  dtraj = fnder(PPTrajectory(spline(tts,xtraj_smooth)));
  qddtraj = dtraj(nq+(1:nq));

  lfoot = findLinkId(r,'l_foot');
  rfoot = findLinkId(r,'r_foot');

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

    lfoot_cpos = terrainContactPositions(r,kinsol,lfoot);
    rfoot_cpos = terrainContactPositions(r,kinsol,rfoot);

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

    rfoottraj = PPTrajectory(pchip(walking_ctrl_data.link_constraints(1).ts,...
                                   walking_ctrl_data.link_constraints(1).poses));
    lfoottraj = PPTrajectory(pchip(walking_ctrl_data.link_constraints(2).ts,...
                                   walking_ctrl_data.link_constraints(2).poses));

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
  tc_lfoot = getTerrainContactPoints(r,lfoot);
  tc_rfoot = getTerrainContactPoints(r,rfoot);
  for i=1:size(left_foot_steps,2);
    cpos = rpy2rotmat(left_foot_steps(4:6,i)) * tc_lfoot.pts + repmat(left_foot_steps(1:3,i),1,4);
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
    cpos = rpy2rotmat(right_foot_steps(4:6,i)) * tc_rfoot.pts + repmat(right_foot_steps(1:3,i),1,4);
    if all(cpos(3,:)<=0.001)
      plot(cpos(1,[1,2]),cpos(2,[1,2]),'k-','LineWidth',2);
      plot(cpos(1,[1,3]),cpos(2,[1,3]),'k-','LineWidth',2);
      plot(cpos(1,[2,4]),cpos(2,[2,4]),'k-','LineWidth',2);
      plot(cpos(1,[3,4]),cpos(2,[3,4]),'k-','LineWidth',2);
    end
  end

  for i=1:lstep_counter
    cpos = rpy2rotmat(lfoot_steps(4:6,i)) * tc_lfoot.pts + repmat(lfoot_steps(1:3,i),1,4);
    plot(cpos(1,[1,2]),cpos(2,[1,2]),'g-','LineWidth',1.65);
    plot(cpos(1,[1,3]),cpos(2,[1,3]),'g-','LineWidth',1.65);
    plot(cpos(1,[2,4]),cpos(2,[2,4]),'g-','LineWidth',1.65);
    plot(cpos(1,[3,4]),cpos(2,[3,4]),'g-','LineWidth',1.65);
  end

  for i=1:rstep_counter
    cpos = rpy2rotmat(rfoot_steps(4:6,i)) * tc_rfoot.pts + repmat(rfoot_steps(1:3,i),1,4);
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

end
