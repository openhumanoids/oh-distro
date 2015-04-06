%% Add in the appropriate path
path = [getenv('DRC_BASE'),'/software/control/matlab/planners/prone'];
addpath(path);


%% Construct the robot
%needs to be the convex hull model of Atlas to get collision constraints to work out
% Also need to remove some collision groups from being ignored
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_convex_hull.urdf'];
options.floating = true;
robot = RigidBodyManipulator(atlas_urdf,options);
% robot = robot.removeCollisionGroupsExcept({});
data_struct = {};
kpt = KinematicPoseTrajectory(robot,data_struct);
kpt = kpt.useRobotiqHands();
kpt = kpt.addTerrain();
robot = kpt.addCollisionGeometryToRobot(robot);
robot = compile(robot);
% robot = kpt.robot.addLinksToCollisionFilterGroup(kpt.robot.findLinkId('head'),'ignores_ground',1);
% robot = compile(robot);
% kpt = kpt.setRobot(robot);

%% Add the RigidBodyBox
% box_height = 1;
% box = RigidBodyBox([1;2;box_height;]);
% robot = robot.addCollisionGeometryToBody(1,box);
% robot = robot.addVisualGeometryToBody(1,box);
% robot = robot.compile();
% 
% 
% % Extra options for KPT object
% kpt = kpt.useHandGuards();
% kpt = kpt.setQuasiStaticShrinkFactor(0.8);
% kpt = kpt.setMinDistance(0.03);
% [~,kpt] = kpt.addCollisionGeometryToRobot();

% try to make it not end up somewhere crazy at the end
% can either add a constraint on where we need to end up
% kpt = kpt.setConstraintTol(kpt.constraint_err_tol);

%% Nice Robot + visualizer
clear options;
options.floating = true;
options.ignore_self_collisions = true;
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v4/model_minimal_contact.urdf'];
r = RigidBodyManipulator(atlas_urdf,options);
r = r.removeCollisionGroupsExcept({});
% r = r.setTerrain(RigidBodyFlatTerrain);
r = kpt.addVisualContactPoints(r);
% r = r.addVisualGeometryToBody(1,box);
% r = r.addCollisionGeometryToBody(1,box);
r = r.compile();
v = r.constructVisualizer;
v.inspector();
np = r.getNumPositions();
nq = np;
% 
% %% Find an initial sitting pose
% seat_data = load('data_sitting.mat');
% clear options;
% q_nom = seat_data.x_seat(1:np);
% options.enforce_collision = 1;
% options.enforce_contact = 1;
% options.enforce_quasistatic = 1;
% kpt.shrink_factor = 0.4
% contacts = {'l_fpelvis','r_fpelvis','r_foot','l_foot'};
% contact_height = containers.Map();
% height = box_height/2;
% contact_height('l_fpelvis') = height;
% contact_height('r_fpelvis') = height;
% kpt = kpt.setContactHeight(contact_height);
% [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
% info
% infeasible_constraint
% kpt.drawCOM(r,q);
% q_1 = q;
% 
% 
% %% Find a sitting pose further back
% 
% q_nom = q_1;
% pelvis_contacts = {'l_fpelvis','r_fpelvis'};
% position_cost = struct();
% kinsol = r.doKinematics(q_nom);
% options.qs_contacts = {'l_heel','r_heel'};
% 
% for j = 1:numel(pelvis_contacts)
%   name = pelvis_contacts{j};
%   position_cost(j).name = name;
%   pos_old = r.forwardKin(kinsol,kpt.linkId(name),kpt.c(name));
%   pos_new = pos_old;
%   pos_new(1,:)  = pos_new(1,:) - 0.4;
%   position_cost(j).position = pos_new;
% end
% options.no_movement.contacts = {'l_foot','r_foot'};
% options.no_movement.q = q_nom;
% [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,position_cost,q_nom,options);
% info
% infeasible_constraint
% kpt.drawCOM(r,q);
% q_2 = q;
% 
% %% Sitting pose with only feet for COM
% 
% options.no_movement.contacts = contacts;
% options.no_movement.q = q;
% kpt.shrink_factor = 0.3;
% options.qs_contacts = {'l_foot','r_foot'};
% [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,position_cost,q_nom,options);
% info
% infeasible_constraint
% kpt.drawCOM(r,q);
% q_3 = q;

%% Add the polaris model
polaris_urdf = [getenv('DRC_BASE'),'/software/models/polaris/model_no_rollcage.urdf'];
xyz = [0;0;0];
rpy = [0;0;pi];
r = r.addRobotFromURDF(polaris_urdf,xyz,rpy);
v = r.constructVisualizer();

%% Polaris Dimensions
world = r.getBody(1);
seat = world.collision_geometry{12};
seat_xyz = seat.T(1:3,4);
seat_height = seat.T(3,4) + seat.size(3)/2;

polaris_floor = world.collision_geometry{1};
floor_xyz = polaris_floor.T(1:3,4);
floor_height = polaris_floor.T(3,4) + polaris_floor.size(3)/2;

lcmgl = LCMGLClient;
lcmgl.glColor3f(1,0,0);
seat_sphere_pos = [seat_xyz(1:2);seat_height];
lcmgl.sphere(seat_sphere_pos,0.02,20,20);

floor_sphere_pos = [floor_xyz(1:2);floor_height];
lcmgl.sphere(floor_sphere_pos,0.02,20,20);
lcmgl.switchBuffers;


%% Polaris sitting pose
seat_data = load('data_sitting.mat');
clear options;
q_nom = seat_data.x_polaris(1:np);
options.enforce_collision = 1;
options.enforce_contact = 1;
options.enforce_quasistatic = 1;
kpt.shrink_factor = 0.4;
contacts = {'l_fpelvis','r_fpelvis','r_foot','l_foot'};
contact_height = containers.Map();
contact_height('l_fpelvis') = seat_height;
contact_height('r_fpelvis') = seat_height;
contact_height('r_foot') = floor_height;
contact_height('l_foot') = floor_height;
kpt = kpt.setContactHeight(contact_height);

% need to add foot/pelvis y position constraints
constraints = {};
lb_y = floor_xyz(2) + polaris_floor.size(2)/2 + 0.1;
lb =[nan;lb_y;nan];
ub = [nan;nan;nan];
lb = repmat(lb,1,size(kpt.c('l_foot'),2));
ub = repmat(ub,1,size(kpt.c('l_foot'),2));
lfoot_y = WorldPositionConstraint(kpt.robot,kpt.linkId('l_foot'),kpt.c('l_foot'),...
  lb,ub);
rfoot_y = WorldPositionConstraint(kpt.robot,kpt.linkId('r_foot'),kpt.c('r_foot'),...
  lb,ub);

ub_y = seat_xyz(2) + seat.size(2)/2 - 0.1;
lb =[nan;nan;nan];
ub = [nan;ub_y;nan];
lb = repmat(lb,1,size(kpt.c('l_fpelvis'),2));
ub = repmat(ub,1,size(kpt.c('r_fpelvis'),2));
lpelvis_y = WorldPositionConstraint(kpt.robot,kpt.linkId('l_fpelvis'),kpt.c('l_fpelvis'),...
  lb,ub);
rpelvis_y = WorldPositionConstraint(kpt.robot,kpt.linkId('r_fpelvis'),kpt.c('r_fpelvis'),...
  lb,ub);

options.constraints = {lfoot_y,rfoot_y,rpelvis_y,lpelvis_y};
[q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
info
infeasible_constraint
kpt.drawCOM(r,q);
q_4 = q;


%% Polaris Sitting foot support
options.qs_contacts = {'l_foot','r_foot'};
options.no_movement.contacts = contacts;
kpt.shrink_factor = 0.5;
options.no_movement.q = q_4;
q_nom = seat_data.x_polaris_feet;
q_nom = q_nom(1:nq);
[q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
info
infeasible_constraint
kpt.drawCOM(r,q);
q_5 = q;


















