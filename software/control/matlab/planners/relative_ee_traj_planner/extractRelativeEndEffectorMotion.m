function ee_pose_relative = extractRelativeEndEffectorMotion(ee_name,X,r,ref_link_name)
  if nargin < 3
    r = Atlas();
  end
  if nargin < 4
    ref_link_name = ee_name;
  end
  nq = r.getNumDOF();
  ee_idx = r.findLinkInd(ee_name);
  ref_link_idx = r.findLinkInd(ref_link_name);
  nt = size(X,2);

  kinsol = doKinematics(r,X(1:nq,1));
  ref_pose_0 = forwardKin(r,kinsol,ref_link_idx,[0;0;0],2);
  xyz_0 = ref_pose_0(1:3);
  quat_0 = ref_pose_0(4:7);
  ee_pose_relative = zeros(7,nt);
  for i = 1:nt
    kinsol = doKinematics(r,X(1:nq,i));
    ee_pose = forwardKin(r,kinsol,ee_idx,[0;0;0],2);
    xyz = ee_pose(1:3);
    quat = ee_pose(4:7);
    ee_pose_relative(:,i) = [quatRotateVec(quatConjugate(quat_0),xyz - xyz_0); quatDiff(quat_0,quat)];
  end
end
