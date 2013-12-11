function X = relativeEndEffectorTrajPlanner(r,q0,ee_name,ee_pose_relative,options)
  if nargin < 5, options = struct(); end
  if ~isfield(options,'free_back'), options.free_back = false; end
  if ~isfield(options,'pos_ltol'), options.pos_ltol = -0.05; end
  if ~isfield(options,'pos_utol'), options.pos_utol = 0.05; end
  if ~isfield(options,'quat_tol'), options.quat_tol = sin(10*pi/180)^2; end
  if ~isfield(options,'free_joints'), options.free_joints = []; end
  if ~isfield(options,'ref_link_name'), options.ref_link_name = ee_name; end
  nq = r.getNumDOF();
  nt = size(ee_pose_relative,2);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setDebug(true);
  ikoptions = ikoptions.setMajorIterationsLimit(5000);
  ee_idx = r.findLinkInd(ee_name);
  switch ee_name
    case 'l_hand'
      free_joints = findJointIndices(r,'l_arm');
    case 'r_hand'
      free_joints = findJointIndices(r,'r_arm');
    otherwise
      error('relativeEndEffectorTrajPlanner:badEEName',...
        '%s is not a supported end-effector name',ee_name);
  end
  if options.free_back
    free_joints = [free_joints; findJointIndices(r,'back')];
  end
  if ~isempty(options.free_joints)
    free_joints = [free_joints; options.free_joints];
  end
  fixed_joints = setdiff(1:nq,free_joints)';

  fixed_joints_constraint = PostureConstraint(r);
  fixed_joints_constraint = ...
    fixed_joints_constraint.setJointLimits(fixed_joints,q0(fixed_joints),q0(fixed_joints));
  q_data = zeros(nq,nt);
  q_seed = q0;
  q_nom = q0;
  for i = 1:nt
    [ee_pose,ref_pose] = relativeEEPoseToWorldEEPose(options.ref_link_name,ee_pose_relative(:,i),q0,r);
    xyz_rel = ee_pose_relative(1:3,i);
    xyz = ee_pose(1:3);
    quat = ee_pose(4:7);
    P = ref_pose(1:3);
    R = quat2rotmat(ref_pose(4:7));
    T = [R,P;zeros(1,3),1];
    hand_xyz_constraint = WorldPositionInFrameConstraint(r,ee_idx,[0;0;0],T,...
      xyz_rel+options.pos_ltol,xyz_rel+options.pos_utol);
    hand_quat_constraint = WorldQuatConstraint(r,ee_idx,quat,options.quat_tol);
    [q_data(:,i),info] = inverseKin(r,q_seed,q_nom,fixed_joints_constraint, ...
                                hand_xyz_constraint, ...
                                hand_quat_constraint,ikoptions);
    if info > 4 
      display(info); 
    else
      q_seed = q_data(:,i);
    end;
  end
  X = [q_data;zeros(nq,nt)];
  % Smooth the output
  for i = 1:2*nq, X(i,:) = smooth(X(i,:)'); end
end
