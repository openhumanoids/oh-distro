%% Listens for commited plans, and enacts them
fprintf('Setup...\n');
path_handle = addpathTemporary(fullfile(getDrakePath, 'examples', 'IRB140'));

dt = 0.005;
options.dt = dt;
options.floating = false;
options.base_offset = [0;0;0]; %[-0.5, 0, 1.5]'; %For now positions on ground
options.base_rpy = [-pi/2, 0, 0]';
options.ignore_self_collisions = true;
options.collision = false;
options.hands = 'robotiq_weight_only';
r = IRB140(fullfile(getDrakePath, 'examples', 'IRB140', 'urdf', 'irb_140.urdf'), options);

gravcomp = GravityCompensationBlock(r);

% Set up LCM listener for committed plans
joint_names = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'};
listener_plan = RobotPlanListener('COMMITTED_ROBOT_PLAN',false,joint_names);
robot_state = drcFrames.AtlasState(r);
robot_state.subscribe(robot_state.defaultChannel);

lc = lcm.lcm.LCM.getSingleton();
coordnames = r.getInputFrame.coordinates;
joint_names_cache = cell(length(coordnames), 1);
 % (this is the part that we really don't want to run repeatedly...)
for i=1:length(coordnames)
  joint_names_cache(i) = java.lang.String(coordnames(i));
end
      
q_des = [-0.08, 0.69, -0.4, 0.09, 1.02, 1.8, zeros(1, 6)].';
t_start = 0;
traj_follow = PPTrajectory(foh([t_start, t_start+1], [q_des, q_des]));
last_t = 0;
inte = zeros(6, 1);
while (1)
  [x_traj,t_traj] = listener_plan.getNextMessage(5);
  if ~isempty(x_traj)
    fprintf('Got a new traj!\n');
    traj_follow = PPTrajectory(foh(t_traj+last_t, x_traj));
  end
  
  [x,tsim] = robot_state.getNextMessage(5);  % timeout is in msec - should be safely bigger than e.g. a 200Hz input rate
  if isempty(x) continue; end
  
 
  % p-d on desired state until I write this better
  K_P = 0.3*[10000; 500; 250; 50; 25; 1];
  K_D = 0.3*[1000; 500; 250;  50; 25; 1];
  if (size(traj_follow.eval(tsim), 2) == 0)
    asdf
  end
  error = traj_follow.eval(tsim) - x;
  if (tsim - last_t < 0.1)
    inte = inte + error(1:6)*(tsim-last_t);
  end
  efforts = K_P.*error(1:6) - K_D.*x(7:12);
  
  y = gravcomp.output(tsim, 0, x);
  efforts = efforts + y;
  
  robot_command = bot_core.atlas_command_t();
  robot_command.utime = tsim*1E6;
  robot_command.position = zeros(6,1);
  robot_command.velocity = zeros(6,1);
  robot_command.k_q_p = zeros(6,1);
  robot_command.k_q_i = zeros(6,1);
  robot_command.k_qd_p = zeros(6,1);
  robot_command.k_f_p = zeros(6,1);
  robot_command.ff_qd = zeros(6,1);
  robot_command.ff_qd_d = zeros(6,1);
  robot_command.ff_f_d = zeros(6,1);
  robot_command.ff_const = zeros(6,1);
  robot_command.k_effort = zeros(6,1);
  robot_command.desired_controller_period_ms = 0;
  
  robot_command.num_joints = 6;
  robot_command.joint_names = joint_names_cache;
  robot_command.effort = efforts;
  lc.publish('ATLAS_COMMAND', robot_command);
   last_t = tsim;
end