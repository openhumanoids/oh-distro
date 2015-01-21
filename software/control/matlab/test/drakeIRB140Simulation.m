%% load in
fprintf('Setup...\n');
dt = 0.001;
options.dt = dt;
options.floating = false;
options.base_offset = [0;0;0]; %[-0.5, 0, 1.5]'; %For now positions on ground
options.base_rpy = [-pi/2, 0, 0]';
options.ignore_self_collisions = true;
options.hands = 'block_hand';
options.collision = false;
options.hands = 'none';
r = IRB140([getenv('DRC_PATH'),'/models/IRB140/irb_140.urdf'], options);
options_hand = options;
options_hand.hands = 'block_hand';
options_hand.terrain = RigidBodyFlatTerrain();
options_hand.collision = true;
r_hand = IRB140([getenv('DRC_PATH'),'/models/IRB140/irb_140.urdf'], options_hand);
r_hand = r_hand.removeCollisionGroupsExcept({'block_hand'});
r_hand = r_hand.compile();

options_cyl.floating = true;
%won't work until floating joints supported in addRobotFromURDF
%r_hand = r_hand.addRobotFromUgetInitialRDF('table.urdf', [0.75; 0.0; -0.45], [])
%r_hand = r_hand.addRobotFromURDF('drill_box.urdf', [0.5; 0.07; 0.025], [], options_cyl);
%r_hand = compile(r_hand);


v=r.constructVisualizer();

%% initial config
% x0_hand = r_hand.getInitialState();
x0 = r.getInitialState();
x0_hand = r_hand.getInitialState();
x0_hand(1:length(x0)) = x0;
x_hand = r_hand.resolveConstraints(x0_hand);

% LCM interpret in
outs(1).system = 2;
outs(1).output = 1;
% outs(2).system = 2;
% outs(2).output = 2;
lcmInputBlock = LCMInputFromIRB140AtlasCommandBlock(r_hand,[],options);
sys = mimoFeedback(lcmInputBlock, r_hand, [], [], [], outs);

broadcast_opts = options;
broadcast = IRB140LCMBroadcastBlock(r, r_hand, broadcast_opts);
sys = mimoCascade(sys, broadcast);

%% simulate
v=r_hand.constructVisualizer();
v.display_dt = 0.001;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
clear output_select;
output_select(1).system=1;
output_select(1).output=1;
% output_select(2).system=1;
% output_select(2).output=2;
%output_select(3).system=1;
%output_select(3).output=3;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

traj = simulate(sys,[0 Inf],x0_hand);

% This doesn't see hand movements. Why?
playback(v,traj,struct('slider',true));