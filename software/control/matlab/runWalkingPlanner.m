function runWalkingPlanner(lcm_plan, goal_x, goal_y, goal_yaw)

if nargin < 4; goal_yaw = 0.0; end
if nargin < 3; goal_y = 0.0; end
if nargin < 2; goal_x = 2.0; end
if nargin < 1; lcm_plan = true; end

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
options.dt = 0.001;
r = Atlas('../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);

while true 

  d = load('data/atlas_fp.mat');
  xstar = d.xstar;
  r = r.setInitialState(xstar);
  state_frame = getStateFrame(r);
  state_frame.subscribe('EST_ROBOT_STATE');

  nq = getNumDOF(r);
  x0 = xstar;

  pose = [goal_x;goal_y;0;0;0;goal_yaw];

  if ~lcm_plan
    [rfoot, lfoot] = planFootsteps(r, x0, pose, struct('plotting', true, 'interactive', false));
  else
    footstep_plan_listener = FootstepPlanListener('COMMITTED_FOOTSTEP_PLAN');
    disp('Listening for footstep plans...');
    waiting = true;
    foottraj = [];


    while waiting
      foottraj = footstep_plan_listener.getNextMessage(0);
      if (~isempty(foottraj))
        disp('footstep plan received.');
        waiting = false;
      end
      [x,~] = getNextMessage(state_frame,10);
      if (~isempty(x))
        %%% TEMP HACK FOR QUAL 1 %%%
        x(3) = x(3)-1.0;
        %%% TEMP HACK FOR QUAL 1 %%%
        x0=x;
      end
    end

    rfoot = foottraj(1:6,find(foottraj(15,:)==1));
    lfoot = foottraj(1:6,find(foottraj(15,:)==0));
  end

  q0 = x0(1:nq);
  kinsol = doKinematics(r,q0);

  [zmptraj,foottraj,~,supptraj] = planZMPandHeelToeTrajectory(r, q0, rfoot, lfoot, 1.3);
  zmptraj = setOutputFrame(zmptraj,desiredZMP);

  % construct ZMP feedback controller
  com = getCOM(r,kinsol);
  limp = LinearInvertedPendulum(com(3));
  % get COM traj from desired ZMP traj
  comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);

  if 1
    [~,V] = ZMPtracker(limp,zmptraj);
  end

  % time spacing of samples for IK
  ts = 0:0.1:zmptraj.tspan(end);

  % create desired joint trajectory
  cost = Point(r.getStateFrame,1);
  cost.pelvis_x = 0;
  cost.pelvis_y = 0;
  cost.pelvis_z = 0;
  cost.pelvis_roll = 1000;
  cost.pelvis_pitch = 1000;
  cost.pelvis_yaw = 0;
  cost.back_mby = 100;
  cost.back_ubx = 100;
  cost = double(cost);
  options = struct();
  options.Q = diag(cost(1:r.getNumDOF));
  options.q_nom = q0;

  rfoot_body = r.findLink('r_foot');
  lfoot_body = r.findLink('l_foot');

  disp('Computing robot plan...');
  % v = r.constructVisualizer;
  % v.display_dt = 0.05;
  htraj = [];
  for i=1:length(ts)
    t = ts(i);
    if (i>1)
      q(:,i) = inverseKin(r,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],foottraj.right.orig.eval(t),lfoot_body,[0;0;0],foottraj.left.orig.eval(t),options);
    else
      q = q0;
    end
    com = getCOM(r,q(:,i));
    htraj = [htraj com(3)];
  %   v.draw(t,q(:,i));
  end
  qtraj = PPTrajectory(spline(ts,q));
  htraj = PPTrajectory(spline(ts,htraj));
  
  % publish robot plan
  disp('Publishing robot plan...');
  xtraj = zeros(getNumStates(r),length(ts));
  xtraj(1:getNumDOF(r),:) = q;
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
  plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
  plan_pub.publish(ts,xtraj);

  if 0 % do proper TV linear system approach
    disp('Computing ZMP controller...');
    limp = LinearInvertedPendulum(htraj);
    [~,V] = ZMPtracker(limp,zmptraj);
  end

  hddot = fnder(htraj,2);

  disp('Waiting for robot plan confirmation...');
  plan_listener = RobotPlanListener('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
  waiting = true;
  while waiting
    rplan = plan_listener.getNextMessage(100);
    if (~isempty(rplan))
      % for now don't do anything with it, just use it as a flag
      disp('Plan confirmed. Executing...');
      waiting = false;
    end
  end

  walking_pub = WalkingPlanPublisher('COMMITTED_WALKING_PLAN');
  walking_pub.publish(0,struct('Straj',V.S,'htraj',htraj,'hddtraj',hddot,'qtraj',qtraj,'supptraj',supptraj));

end

end
