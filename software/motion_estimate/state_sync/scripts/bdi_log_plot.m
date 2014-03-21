close all; clear all
lc = lcm.lcm.LCM.getSingleton();
load('/home/mfallon/data/atlas/2013-08-02-sample-from-bdi/20130802_171021_walk.mat');

joints={'j_back',	'bkx',	'back_ubx';
'j_back',	'bky',	'back_lbz';
'j_back',	'bkz',	'back_mby';
'j_neck',	'ry',	   'neck_ay';
'j_lleg',	'akx',	'l_leg_lax';
'j_lleg',	'aky',	'l_leg_uay';
'j_lleg',	'hpx',	'l_leg_mhx';
'j_lleg',	'hpy',	'l_leg_lhy';
'j_lleg',	'hpz',	'l_leg_uhz';
'j_lleg',	'kny',	'l_leg_kny';
'j_rleg',	'akx',	'r_leg_lax';
'j_rleg',	'aky',	'r_leg_uay';
'j_rleg',	'hpx',	'r_leg_mhx';
'j_rleg',	'hpy',	'r_leg_lhy';
'j_rleg',	'hpz',	'r_leg_uhz';
'j_rleg',	'kny',	'r_leg_kny';
'j_larm',	'elx',	'l_arm_elx';
'j_larm',	'ely',	'l_arm_ely';
'j_larm',	'shx',	'l_arm_shx';
'j_larm',	'shy',	'l_arm_usy';
'j_larm',	'wrx',	'l_arm_mwx';
'j_larm',	'wry',	'l_arm_uwy';
'j_rarm',	'elx',	'r_arm_elx';
'j_rarm',	'ely',	'r_arm_ely';
'j_rarm',	'shx',	'r_arm_shx';
'j_rarm',	'shy',	'r_arm_usy';
'j_rarm',	'wrx',	'r_arm_mwx';
'j_rarm',	'wry',	'r_arm_uwy'};

d=struct();
for j=1:size(var_list,1)
  v =var_list(j,:);
  v_split = strread(v,'%s','delimiter','.');
  if size(v_split,1) == 2 
    d.(v_split{2}) = data(j,:);
  elseif size(v_split,1) == 3 
    d.(v_split{2}).(v_split{3}) = data(j,:);
  elseif size(v_split,1) == 4
    d.(v_split{2}).(v_split{3}).(v_split{4}) = data(j,:);
  else
    keyboard
  end
end


% Create a basic state:
s = drc.robot_state_t();
s.num_joints = size(joints,1);
s.joint_name = joints(:,3);
ft = drc.force_torque_t();
s.force_torque = ft;
twist = drc.twist_t();
v = drc.vector_3d_t();
twist.linear_velocity = v;
twist.angular_velocity = v;
s.twist = twist;


p = bot_core.pose_t();
m = drc.position_3d_t();
r = drc.quaternion_t();

i_start = find(d.timestamp == 1267943165252738);
for i=i_start:1:size(data,2)
  jt.effort=[]; jt.position=[]; jt.velocity=[];
  for j=1:size(joints,1)
    jt.effort(j) = d.(joints{j,1}).(joints{j,2}).f(i);
    jt.position(j) = d.(joints{j,1}).(joints{j,2}).q(i);
    jt.velocity(j) = d.(joints{j,1}).(joints{j,2}).qd(i);
  end
  s.joint_effort = jt.effort;
  s.joint_position = jt.position;
  s.joint_velocity = jt.velocity;
  
  v.x = d.pose_est.position.x(i); v.y = d.pose_est.position.y(i);
  v.z = d.pose_est.position.z(i);
  r.w = d.filtered_imu.orientation_estimate.qw(i);
  r.x = d.filtered_imu.orientation_estimate.qx(i);
  r.y = d.filtered_imu.orientation_estimate.qy(i);
  r.z = d.filtered_imu.orientation_estimate.qz(i);
  m.translation = v; m.rotation = r;
  s.pose = m;
  s.utime =d.timestamp(i);
  lc.publish('EST_ROBOT_STATE', s);
  
  p.utime = d.timestamp(i);
  p.pos = [d.pose_est.position.x(i), d.pose_est.position.y(i), d.pose_est.position.z(i)];
  p.orientation = [d.filtered_imu.orientation_estimate.qw(i),...
    d.filtered_imu.orientation_estimate.qx(i),...
    d.filtered_imu.orientation_estimate.qy(i),...
    d.filtered_imu.orientation_estimate.qz(i)];
  lc.publish('POSE_HEAD',p);
  pause(0.002)
end
keyboard
