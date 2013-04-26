function playCannedPlan(biped, name, x0)
  load('canned_plans.mat')

  for n = {'htraj', 'lfoottraj', 'rfoottraj'}
    traj_name = n{1};
    old_frame.(traj_name) = canned_plans.(name).(traj_name).getOutputFrame();
    new_frame.(traj_name) = CoordinateFrame([traj_name, '_out'], old_frame.(traj_name).dim, old_frame.(traj_name).prefix, old_frame.(traj_name).coordinates);
  end

  R = quat2rotmat(rpy2quat(x0(4:6)));  
  A = sparse([[R, zeros(3)]; [zeros(3), eye(3)]]);
  b = x0(1:6);

  msg_footsteps = canned_plans.(name).footsteps;
  for j = 1:length(msg_footsteps)
    msg_footsteps(j).pos = A * msg_footsteps(j).pos + b;
  end
  biped.publish_footstep_plan(msg_footsteps, 0, 1);

  nq = getNumDOF(biped);
  q0 = x0(1:nq);
  kinsol = doKinematics(biped,q0);

  [zmptraj,foottraj, supptraj] = planInitialZMPTraj(biped, q0, msg_footsteps);
  zmptraj = setOutputFrame(zmptraj,desiredZMP);
  ts = 0:0.08:zmptraj.tspan(end);

  % construct ZMP feedback controller
  com = getCOM(biped,kinsol);
  limp = LinearInvertedPendulum(com(3));
  % get COM traj from desired ZMP traj
  comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);
  [~,V] = ZMPtracker(limp,zmptraj);

  transforms = struct('htraj', AffineTransform(old_frame.htraj, new_frame.htraj, 1, x0(3)),...
    'lfoottraj', AffineTransform(old_frame.lfoottraj, new_frame.lfoottraj, A, b),...
    'rfoottraj', AffineTransform(old_frame.rfoottraj, new_frame.rfoottraj, A, b));

  for n = {'htraj','lfoottraj', 'rfoottraj'}
    addTransform(canned_plans.(name).(n{1}).getOutputFrame(), transforms.(n{1}));
    msg.(n{1}) = canned_plans.(name).(n{1}).inFrame(new_frame.(n{1}));
  end


  msg.hddtraj = fnder(msg.htraj, 2);
  msg.supptraj = supptraj;
  msg.S = V.S;
  msg.s1 = V.s1;
  msg.comtraj = comtraj;

  walking_pub = WalkingPlanPublisher('COMMITTED_WALKING_PLAN');
  walking_pub.publish(0,msg);



