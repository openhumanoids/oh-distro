function sendNewHandPose(pose)
  CHANNEL_NAME = 'REACH_TARGET_POSE';
  utime = get_timestamp_now();
  lc = lcm.lcm.LCM.getSingleton();
  msg = drc.update_t();
  msg.frame = sprintf('CAMERA%sHAND', LR);
  msg.relative_to = 'local';
  msg.trans() = pose(1:3)';
  msg.quat() = pose(4:7)';
  lc.publish(CHANNEL_NAME, msg);
end
