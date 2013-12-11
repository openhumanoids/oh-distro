function lcmglDrawRelativeEndEffectorPose(ee_name,ee_pose_relative,x0,r,lcmgl,switch_buffers)
  if nargin < 4, r = Atlas(); end;
  if nargin < 5 
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'disp_rel_ee_motion');
  end
  if nargin < 6
    switch_buffers = false;
  end
  ee_idx = r.findLinkInd(ee_name);
  nq = r.getNumDOF();
  [ee_pose,ee_pose_0] = relativeEEPoseToWorldEEPose(ee_name,ee_pose_relative,x0,r);
  xyz = ee_pose(1:3);
  quat = ee_pose(4:7);
  xyz_0 = ee_pose_0(1:3);
  quat_0 = ee_pose_0(4:7);
  T = [quat2rotmat(quat), xyz; zeros(1,3), 1];
  lcmgl.glColor3f(0,0,0);
  lcmgl.sphere(xyz_0,0.02,20,20);
  lcmgl.glColor3f(1,0,1);
  lcmgl.sphere(xyz,0.02,20,20);
  lcmglDrawHT(lcmgl,T);
  r.getManipulator().drawLCMGLAxes(lcmgl,x0(1:nq),ee_idx);
  if switch_buffers
    lcmgl.switchBuffers();
  end
end
