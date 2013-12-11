function displayRelativeEndEffectorMotion(ee_name,ee_pose_relative,x0,r,lcmgl)
  if nargin < 4, r = Atlas(); end;
  if nargin < 5 
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'disp_rel_ee_motion');
    local_lcmgl = true;
  else
    local_lcmgl = false;
  end
  nt = size(ee_pose_relative,2);
  %nq = r.getNumDOF();
  %kinsol = doKinematics(r,x0(1:nq));
  %ee_idx = r.findLinkInd(ee_name);
  %ee_pose_0 = forwardKin(r,kinsol,ee_idx,[0;0;0],2);
  %xyz_0 = ee_pose_0(1:3);
  %quat_0 = ee_pose_0(4:7);
  for i = 1:nt
    %quat_diff = ee_pose_relative(4:7,i);
    %quat = quatProduct(quat_0,quat_diff);
    %xyz = xyz_0 + quatRotateVec(quat_0,ee_pose_relative(1:3,i));
    %T = [quat2rotmat(quat), xyz; zeros(1,3), 1];
    %lcmglDrawHT(lcmgl,T);
    lcmglDrawRelativeEndEffectorPose(ee_name,ee_pose_relative(:,i),x0,r,lcmgl)
  end
  if local_lcmgl, lcmgl.switchBuffers(); end;
