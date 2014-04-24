function sendNewCampose(campose, LR)
  CHANNEL_NAME = 'REACH_TARGET_POSE';
  utime = get_timestamp_now();
  lc = lcm.lcm.LCM.getSingleton();
  msg = bot_frames.update_t();
  msg.utime = utime;
  msg.frame = sprintf('CAMERA%sHAND',LR);
  msg.relative_to = 'local';
  msg.trans = campose(1:3,1)';
  msg.quat = campose(4:7,1)';
  lc.publish(CHANNEL_NAME, msg);
end
