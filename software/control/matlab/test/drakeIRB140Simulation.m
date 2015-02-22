function drakeIRB140Simulation(visualize)
if nargin < 1
  visualize = 1;
end

%% load in
fprintf('Setup...\n');
path_handle = addpathTemporary(fullfile(getDrakePath, 'examples', 'IRB140'));

dt = 0.00333;
options.dt = dt;
options.floating = false;
options.base_offset = [0;0;0]; %[-0.5, 0, 1.5]'; %For now positions on ground
options.base_rpy = [-pi/2, 0, 0]';
options.ignore_self_collisions = true;
options.collision = false;
options.hands = 'robotiq_weight_only';
r = IRB140(fullfile(getDrakePath, 'examples', 'IRB140', 'urdf', 'irb_140.urdf'), options);
options_hand = options;
options_hand.hands = 'robotiq';
options_hand.terrain = RigidBodyFlatTerrain();
options_hand.collision = false;
r_hand = IRB140(fullfile(getDrakePath, 'examples', 'IRB140', 'urdf', 'irb_140.urdf'), options_hand);



hand_coll_links = {'palm'; 'finger_1_link_1'; 'finger_1_link_2'; 'finger_1_link_3';
   'finger_2_link_1'; 'finger_2_link_2'; 'finger_2_link_3'; 'finger_middle_link_1'; 'finger_middle_link_2'; 'finger_middle_link_3'};
box_link = 'drill_box';
r_hand = r_hand.addLinksToCollisionFilterGroup(hand_coll_links, 'default', 2); 
r_hand = r_hand.addLinksToCollisionFilterGroup(box_link, 'default', 3); 
r_hand = r_hand.removeCollisionGroupsExcept({'default'});

options_cyl.floating = true;
%won't work until floating joints supported in addRobotFromURDF
%r_hand = r_hand.addRobotFromUgetInitialRDF('table.urdf', [0.75; 0.0; -0.45], [])
r_hand = r_hand.addRobotFromURDF('drill_box.urdf', [0.5; 0.07; 0.025], [], options_cyl);

%% initial config
% x0_hand = r_hand.getInitialState();
x0 = r.getInitialState();
x0_hand = r_hand.getInitialState();
x0_hand(1:length(x0)) = x0;
x0_hand = r_hand.resolveConstraints(x0_hand);

% LCM interpret in
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
outs(3).system = 2;
outs(3).output = 3;
lcmInputBlock = LCMInputFromIRB140AtlasCommandBlock(r_hand,r,options);
sys = mimoFeedback(lcmInputBlock, r_hand, [], [], [], outs);

lcmHandInputBlock = LCMInputFromRobotiqCommandBlock(r_hand,options);
sys = mimoFeedback(lcmHandInputBlock, sys, [], [], [], outs);

broadcast_opts = options;
broadcast = IRB140LCMBroadcastBlock(r_hand, r, broadcast_opts);
sys = mimoCascade(sys, broadcast);

%% simulate
if (visualize)
  v=r_hand.constructVisualizer();
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  clear output_select;
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  output_select(3).system=1;
  output_select(3).output=3;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end

traj = simulate(sys,[0 Inf],x0_hand);

% This doesn't see hand movements. Why?
playback(v,traj,struct('slider',true));