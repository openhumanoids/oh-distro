%% Add in the appropriate path
path = [getenv('DRC_BASE'),'/software/control/matlab/planners/prone'];
addpath(path);


%% Construct the robot + Box
%needs to be the convex hull model of Atlas to get collision constraints to work out
% Also need to remove some collision groups from being ignored
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(atlas_urdf,options);
box_height = 1;
box = RigidBodyBox([1;2;box_height;]);
robot = robot.addCollisionGeometryToBody(1,box);
robot = robot.addVisualGeometryToBody(1,box);
robot = robot.compile();
robot = compile(robot);
% robot = robot.removeCollisionGroupsExcept({});
data_struct = {};
kpt = KinematicPoseTrajectory(robot,data_struct);
kpt = kpt.useRobotiqHands();
kpt = kpt.addTerrain();
[~,kpt] = kpt.addCollisionGeometryToRobot();


%% Nice Robot + visualizer
clear options;
options.floating = true;
options.ignore_self_collisions = true;
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_minimal_contact.urdf'];
r = RigidBodyManipulator(atlas_urdf,options);
r = r.removeCollisionGroupsExcept({});
r = r.setTerrain(RigidBodyFlatTerrain);
r = kpt.addVisualContactPoints(r);
r = r.addVisualGeometryToBody(1,box);
r = r.compile();
v = r.constructVisualizer;
v.inspector();
np = r.getNumPositions();
nq = np;

%% Torque Constraint
joint_names = kpt.robot.getPositionFrame.coordinates;
idx_arm = ~cellfun('isempty',strfind(joint_names,'arm'));
% idx_back = ~cellfun('isempty',strfind(joint_names,'back'));
% idx = or(idx_arm,idx_back);
idx = idx_arm;
names_arm_back = joint_names(idx);
pmin = Point(kpt.robot.getInputFrame,r.umin*0.4);
pmax = Point(kpt.robot.getInputFrame,r.umax*0.4);


lb = zeros(length(names_arm_back),1);
ub = lb;
joint_idx = zeros(length(names_arm_back),1);




for j = 1:length(joint_idx)
  name = names_arm_back{j};
  joint_idx(j) = kpt.robot.findPositionIndices(name);
  lb(j) = pmin.([name,'_motor']);
  ub(j) = pmax.([name,'_motor']);
end

torque_constraint = GravityCompensationTorqueConstraint(kpt.robot,joint_idx,lb,ub);

%% Constraints on feet/pelvis positions
lb_x = box.T(1,4) + box.size(1)/2 + 0.1;
lb =[lb_x;nan;nan];
ub =[nan;nan;nan];
lb = repmat(lb,1,size(kpt.c('l_foot'),2));
ub = repmat(ub,1,size(kpt.c('l_foot'),2));
lfoot_x = WorldPositionConstraint(kpt.robot,kpt.linkId('l_foot'),kpt.c('l_foot'),...
  lb,ub);
rfoot_x = WorldPositionConstraint(kpt.robot,kpt.linkId('r_foot'),kpt.c('r_foot'),...
  lb,ub);

ub_x = box.T(1,4) + box.size(1)/2 - 0.15;
lb =[nan;nan;nan];
ub = [ub_x;nan;nan];
lb = repmat(lb,1,size(kpt.c('l_fpelvis'),2));
ub = repmat(ub,1,size(kpt.c('r_fpelvis'),2));
lpelvis_x = WorldPositionConstraint(kpt.robot,kpt.linkId('l_fpelvis'),kpt.c('l_fpelvis'),...
  lb,ub);
rpelvis_x = WorldPositionConstraint(kpt.robot,kpt.linkId('r_fpelvis'),kpt.c('r_fpelvis'),...
  lb,ub);
constraints = {lfoot_x,rfoot_x,lpelvis_x,rpelvis_x,torque_constraint};

%% Nominal pose
atlas_fp = load([getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v4_fp.mat']);
xstar = atlas_fp.xstar;
qstar = xstar(1:nq);
arm_idx = kpt.robot.findPositionIndices('arm');


%% Find an initial sitting pose
seat_data = load('data_sitting.mat');
clear options;
options.constraints = constraints;
q_nom = seat_data.x_seat(1:np);
q_nom(arm_idx) = qstar(arm_idx);
options.enforce_collision = 1;
options.enforce_contact = 1;
options.enforce_quasistatic = 1;
kpt.shrink_factor = 0.4;
contacts = {'l_fpelvis','r_fpelvis','r_foot','l_foot'};
contact_height = containers.Map();
height = box_height/2;
contact_height('l_fpelvis') = height;
contact_height('r_fpelvis') = height;
kpt = kpt.setContactHeight(contact_height);
[q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
info
infeasible_constraint
kpt.drawCOM(r,q);
q_1 = q;


%% Sitting pose with only feet for COM
options.constraints = {lfoot_x,rfoot_x,lpelvis_x,rpelvis_x,torque_constraint};
options.no_movement.contacts = contacts;
options.no_movement.q = q_1;
kpt.shrink_factor = 0.3;
q_nom = seat_data.x_seat_feet(1:nq);
chair_poses = load('chair_poses.mat');
q_nom = chair_poses.x_sit_com_feet(1:nq);
% q_nom(arm_idx) = qstar(arm_idx);
options.qs_contacts = {'l_foot','r_foot'};
[q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
info
infeasible_constraint
kpt.drawCOM(r,q);
q_2 = q;

%% Standing in nominal pose
options_old = options;
clear options;
contacts = {'l_foot','r_foot'};
options.no_movement.contacts = contacts;
options.no_movement.q = q_2;
options.enforce_collision = 1;
options.enforce_contacts = 1;
options.enforce_quasistatic = 1;
atlas_fp = load([getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v4_fp.mat']);
q_nom = atlas_fp.xstar(1:nq);
[q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
info
infeasible_constraint
kpt.drawCOM(r,q);
q_3 = q;


%% Construct the plan we need for simulation/control

q_sol = [q_1,q_2,q_3];
ts = linspace(0,6,3);
qtraj = PPTrajectory(foh(ts,q_sol));
qtraj = qtraj.setOutputFrame(r.getPositionFrame);
v.playback(qtraj,struct('slider',true))

support_times = ts;
supports = struct('bodies',{},'contact_pts',{});
lfoot_pts = kpt.c('l_foot');
rfoot_pts = kpt.c('r_foot');
supports(1).bodies = [kpt.linkId('l_foot'),kpt.linkId('r_foot'),kpt.linkId('r_fpelvis'),kpt.linkId('l_fpelvis')];
supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot'),kpt.c('r_fpelvis'),kpt.c('l_fpelvis')};

supports(2).bodies = [kpt.linkId('l_foot'),kpt.linkId('r_foot')];
supports(2).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};

supports(3) = supports(2);

%% Construct a QPLocomotionPlan
 atlas_version = 4; 
 visualize = false;
 add_hokuyo = true;
 add_hands = 1;
 world_name = '';

% IF YOU WANT MASS EST LOOK HERE
% (when this is more fleshed out this will become
% an argument)
use_mass_est = false;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct a pure atlas model for control
options.visualize = false; %don't use lcmgl lidar visualization... it's very slow
options.atlas_version = atlas_version;
options.floating = true;
options.dt = 0.00333;
options.hokuyo = false;
options.foot_force_sensors = false; % This works (you'll have to change
% LCMBroadcastBlock to broadcast them)
% but is slow right now.
options.obstacles = false; % Replaced by step terrain, though the option still works...
if (add_hands)
  options.hands = 'robotiq_weight_only';
end
r_pure = DRCAtlas([],options);

%%
standup_plan = QPLocomotionPlan.from_standup_traj(r_pure,qtraj,supports,support_times);

%% Save chair standup data

% save('chair_standup_data','q_sol')

%% Test out plan_standup
q0 = q_1;
x0 = [q0;0*q0];
speed = 1;
x0(2) = x0(2) + 0.5;
[qtraj,supports,support_times] = plan_standup(robot,x0,speed);
qtraj = qtraj.setOutputFrame(r.getPositionFrame);
v.playback(qtraj,struct('slider',true));

%% Test plan_sitdown
q0 = q_3;
x0 = [q0;0*q0];
speed = 1;
[qtraj,supports,support_times] = plan_sitdown(robot,x0,speed);
qtraj = qtraj.setOutputFrame(r.getPositionFrame);
v.playback(qtraj,struct('slider',true));












