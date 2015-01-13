function [ee_pose,ref_pose] = relativeEEPoseToWorldEEPose(ref_link_name,ee_pose_relative,x0,r)
  if nargin < 4, r = DRCAtlas(); end
  nq = r.getNumPositions();
  kinsol = doKinematics(r,x0(1:nq));
  ref_idx = r.findLinkId(ref_link_name);
  ref_pose = forwardKin(r,kinsol,ref_idx,[0;0;0],2);
  xyz_0 = ref_pose(1:3);
  quat_0 = ref_pose(4:7);
  quat_diff = ee_pose_relative(4:7);
  quat = quatProduct(quat_0,quat_diff);
  xyz = xyz_0 + quatRotateVec(quat_0,ee_pose_relative(1:3));
  ee_pose = [xyz;quat];
end
