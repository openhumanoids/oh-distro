function [xstar,info,r,ustar,x0,constraints,ikoptions] = randomPose(r,x0,n,v)
  if nargin < 1
    options.floating = true;
    options.dt = 0.001;
    r = Atlas([getenv('DRC_PATH') '/models/mit_gazebo_models/mit_robot_drake/model_param.urdf']);
  end

  l_foot = findLinkInd(r,'l_foot');
  r_foot = findLinkInd(r,'r_foot');
  neck_joint = findJointIndices(r,'neck_ay');
  back_y_joint = findJointIndices(r,'back_bky');

  if nargin < 2
    % set initial state to fixed point
    S_atlas_fp = load([getenv('DRC_PATH') '/control/matlab/data/atlas_fp.mat']);
    x0 = S_atlas_fp.xstar;
  end

  if nargin < 3
    n = 1;
  end
  rbm = r.getManipulator();
  nq = rbm.getNumDOF();

  [joint_limits_min,joint_limits_max] = rbm.getJointLimits();
  ikoptions = IKoptions(rbm);
  ikoptions = ikoptions.setMex(true);
  cost = Point(rbm.getStateFrame(),1);
  cost.base_z = 0;
  cost.base_roll = 10;
  cost.base_pitch = 10;
  ikoptions = ikoptions.setQ(diag(cost(1:nq)));
  l_foot_pts = rbm.body(l_foot).getContactPoints();
  r_foot_pts = rbm.body(r_foot).getContactPoints();
  kinsol = doKinematics(rbm,x0(1:nq));
  l_foot_pts_des = forwardKin(rbm,kinsol,l_foot,l_foot_pts,0);
  r_foot_pts_des = forwardKin(rbm,kinsol,r_foot,r_foot_pts,0);
  freeze_joints = PostureConstraint(rbm);
  freeze_joints = freeze_joints.setJointLimits([neck_joint;back_y_joint],x0([neck_joint,back_y_joint]),x0([neck_joint,back_y_joint]));
  qsc = QuasiStaticConstraint(rbm);
  qsc = qsc.addContact(l_foot,l_foot_pts);
  qsc = qsc.addContact(r_foot,r_foot_pts);
  qsc = qsc.setActive(true);
  qsc = qsc.setShrinkFactor(0.2);
  abcdc = AllBodiesClosestDistanceConstraint(r,0.05,1e3);
  %foot_tol = 0.6;
  % kc_l_foot = WorldPositionConstraint(rbm,l_foot,l_foot_pts,[-foot_tol*ones(2,size(l_foot_pts,2));zeros(1,size(l_foot_pts,2))],[foot_tol*ones(2,size(l_foot_pts,2));zeros(1,size(l_foot_pts,2))]);
  % kc_r_foot = WorldPositionConstraint(rbm,r_foot,r_foot_pts,[-foot_tol*ones(2,size(r_foot_pts,2));zeros(1,size(r_foot_pts,2))],[foot_tol*ones(2,size(r_foot_pts,2));zeros(1,size(r_foot_pts,2))]);
  kc_l_foot = WorldPositionConstraint(rbm,l_foot,l_foot_pts,l_foot_pts_des,l_foot_pts_des);
  kc_r_foot = WorldPositionConstraint(rbm,r_foot,r_foot_pts,r_foot_pts_des,r_foot_pts_des);
  constraints = {qsc,abcdc,kc_l_foot,kc_r_foot,freeze_joints};
  %x_nom = Point(rbm.getStateFrame(), ...
    %(x0 + [rand(nq,1).*(joint_limits_max - joint_limits_min) + joint_limits_min; ...
    %zeros(nq,1)])/2);
  xstar = cell(1,n);
  ustar = cell(1,n);
  info = zeros(1,n);
  i = 1;
  while i <= n
    x_nom = Point(rbm.getStateFrame(), ...
      [rand(nq,1).*(joint_limits_max - joint_limits_min) + joint_limits_min; ...
      zeros(nq,1)]);
    x_nom.base_x = 0;
    x_nom.base_y = 0;
    x_nom.base_z = 0.8;
    x_nom.base_roll = pi/10*(2*rand-1);
    x_nom.base_pitch = pi/10*(2*rand-1);
    x_nom.base_yaw = 0;
    [q,info(i)] = inverseKin(rbm,x_nom(1:nq),x_nom(1:nq),constraints{:},ikoptions);
    if info(i) < 10
      xstar{i} = inFrame(Point(rbm.getStateFrame(),[q;zeros(size(q))]),r.getStateFrame());
      [~,C,B] = r.manipulatorDynamics(q,zeros(nq,1));
      ustar{i} = Point(rbm.getInputFrame(),B\C);
      if nargin > 3
        v.draw(0,xstar{i});
      end
      i = i + 1;
    else
      display('IK failure. Retrying ...');
    end
  end
end
