function atlasDataCollector
global ts cop_knots com_knots zmp_knots pelvis_v lfoot_knots rfoot_knots
% load robot model
r = DRCAtlas();
d=load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar=d.xstar;
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = drcFrames.AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');
force_torque_frame = drcFrames.AtlasForceTorque();
force_torque_frame.subscribe('EST_ROBOT_STATE');

foot_indices_struct.l_foot_fz_idx = find(strcmp('l_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.l_foot_tx_idx = find(strcmp('l_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.l_foot_ty_idx = find(strcmp('l_foot_ty',force_torque_frame.coordinates));
foot_indices_struct.r_foot_fz_idx = find(strcmp('r_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.r_foot_tx_idx = find(strcmp('r_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.r_foot_ty_idx = find(strcmp('r_foot_ty',force_torque_frame.coordinates));

foot_indices_struct.rfoot_ind = r.findLinkId('r_foot');
foot_indices_struct.lfoot_ind = r.findLinkId('l_foot');

nq = getNumPositions(r);

% v = r.constructVisualizer;

t_prev=-1;
qd_prev=-1;
qdd_prev=0;
alpha=0.05;

ts = [];
cop_knots = [];
com_knots = [];
zmp_knots = [];
pelvis_v = [];
lfoot_knots = [];
rfoot_knots = [];
while true
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if t_prev==-1
      dt=0.001;
      t_prev=t;
    elseif t_prev > t
      % skipped backwards in log, reset local state
      t_prev=-1;
      qd_prev=-1;
      qdd_prev=0;      
    else
      dt = t-t_prev;
      t_prev=t;
    end
    
    q = x(1:nq);
    qd = x(nq+(1:nq));

    if qd_prev==-1
      qdd = 0*qd;
    else
      qdd = (1-alpha)*qdd_prev + alpha*(qd-qd_prev)/dt;
    end
    qd_prev = qd;
    qdd_prev = qdd;
    
    kinsol = doKinematics(r,q,false,true);
    cpos_right = terrainContactPositions(r,kinsol,foot_indices_struct.rfoot_ind); 
    cpos_left = terrainContactPositions(r,kinsol,foot_indices_struct.lfoot_ind); 
    cpos = [cpos_right, cpos_left];

    ground_z = min(cpos(3,:));
    
    [com,J] = getCOM(r,kinsol);
    J = J(1:2,:); 

    zmp = getZMP(r,kinsol,qd,qdd,com,J,cpos);

    force_torque = getMessage(force_torque_frame);
    cop = getCOP(force_torque,r,kinsol,foot_indices_struct);

    ts = [ts,t];
    com_knots = [com_knots, com];
    zmp_knots = [zmp_knots, zmp];
    cop_knots = [cop_knots, cop];
    pelvis_v = [pelvis_v, qd(1:3)];

    if (force_torque(foot_indices_struct.l_foot_fz_idx)>400)
      lfoot_pos = forwardKin(r,kinsol, foot_indices_struct.lfoot_ind, [0;0;0]);
      lfoot_knots = [lfoot_knots, lfoot_pos];
    end
    if (force_torque(foot_indices_struct.r_foot_fz_idx)>400)
      rfoot_pos = forwardKin(r,kinsol, foot_indices_struct.rfoot_ind, [0;0;0]);
      rfoot_knots = [rfoot_knots, rfoot_pos];
    end
  end
end

end

function cop = getCOP(force_torque,r,kinsol,foot_indices_struct)
  fz_l = force_torque(foot_indices_struct.l_foot_fz_idx);
  tx_l = force_torque(foot_indices_struct.l_foot_tx_idx);
  ty_l = force_torque(foot_indices_struct.l_foot_ty_idx);
  l_foot_pt = [-ty_l/fz_l; tx_l/fz_l; 0];

  fz_r = force_torque(foot_indices_struct.r_foot_fz_idx);
  tx_r = force_torque(foot_indices_struct.r_foot_tx_idx);
  ty_r = force_torque(foot_indices_struct.r_foot_ty_idx);
  r_foot_pt = [-ty_r/fz_r; tx_r/fz_r; 0];

  lfoot_pos = forwardKin(r,kinsol, foot_indices_struct.lfoot_ind, l_foot_pt);
  rfoot_pos = forwardKin(r,kinsol, foot_indices_struct.rfoot_ind, r_foot_pt);

  cop = (fz_l*lfoot_pos + fz_r*rfoot_pos)/(fz_l+fz_r);
  cop(3) = cop(3)-0.081;
end

function zmp = getZMP(r,kinsol,qd,qdd,com,J,cpos)
  Jdot = forwardJacDot(r,kinsol,0);
  Jdot = Jdot(1:2,:);

  % hardcoding D for ZMP output dynamics
  D = -1.04./9.81*eye(2);

  comdd = Jdot * qd + J * qdd;
  zmp = com(1:2) + D * comdd;
  zmp = [zmp', min(cpos(3,:))]';
end
