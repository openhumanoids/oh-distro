% % KPT_findPose_test

% This file shows how to use the findPose method of the
% KinematicPoseTrajecotry method

%% Load in data (only needed for the seed)
data_KPT = load('data_KPT.mat');
data_back = load('data_back.mat');

%% Construct the robot
%needs to be the convex hull model of Atlas to get collision constraints to work out
% Also need to remove some collision groups from being ignored

% Want to try using the quaternion stuff . . . 
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_convex_hull.urdf'];

% to use quaternions
% options.floating = true;
options.floating = true;

robot = RigidBodyManipulator(atlas_urdf,options);
robot = PoseKinematics.addVisualContactPts(robot);
robot = robot.setTerrain(RigidBodyFlatTerrain());

% tell it not to ignore collisions between the upper legs, this can
% actually happen on v5
% robot = robot.removeFromIgnoredListOfCollisionFilterGroup({'r_uleg'},'l_uleg');
% robot = robot.removeFromIgnoredListOfCollisionFilterGroup({'l_uleg'},'r_uleg');
robot = compile(robot);

%% Initialize KPT object
kpt = KinematicPoseTrajectory(robot);

% kpt.c is a container map of contact points that have been added to the
% robot. To get list of contact points use keys(kpt.c) this would include
% 'l_foot','l_knee' etc.

% kpt.linkId is also a container map, pass it in the name of a contact
% point and it returns the linkId associated with that contact point, i.e.
% kpt.linkId('l_knee') = robot.findLinkId('l_lleg')







%% findPose method

% contacts, cell array of strings specifying which contact points should be
% touching the ground

contacts = {'l_toe','l_knee','r_toe','r_knee'};

% % position_cost, structure array specifying a position 3 x 1 for the given
% % contact point, scale controls the weighting on the cost function for that
% % contact point, 1 is the default.
% position_cost = struct('name',{},'position',{}, 'scale', {});
% position_cost(1).name = 'l_hand';
% position_cost(1).position = [0.061;0.703;0];
% position_cost(1).scale = 1; % optional field
% 
% position_cost(2).name = 'r_hand';
% position_cost(2).position = [-0.0295;-1.09;0];
% position_cost(2).scale = 4; % optional field



% q_nom is the nominal posture it is trying to minize the distance to, the
% standard thing in IK
q_nom_rpy = data_KPT.q_sol(:,1);
% q_nom = q_rpy2quat(robot.getNumPositions()-1,q_nom_rpy);
q_nom = q_nom_rpy;

[q,F,info,infeasible_constraint] = kpt.findPose(contacts,{},q_nom);
v = robot.constructVisualizer;
v.draw(0,q)

% lcmgl = LCMGLClient();
% lcmgl.glColor3f(1,0,0);
% lcmgl.sphere(position_cost(1).position,0.02,20,20);
% lcmgl.switchBuffers;


%% Nice robot and visualizer
% atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_minimal_contact.urdf'];
% 
% options.floating = true;
% robot = RigidBodyManipulator(atlas_urdf,options);
% robot = compile(robot);
% v = robot.constructVisualizer;
% v.inspector
% 


