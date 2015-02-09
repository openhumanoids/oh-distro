% Test class for KinematicPoseTrajectory
data = load('data_KPT_one_knee.mat');

%% Create the robot
% this version of the robot shouldn't have the terrain . . .
% atlas convex hull
atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_convex_hull.urdf'];
options.floating = true;
robot = RigidBodyManipulator(atlas_urdf,options);
robot = PoseKinematics.addVisualContactPts(robot);
robot = robot.setTerrain(RigidBodyFlatTerrain());

% tell it not to ignore collisions between the upper legs, this can
% actually happen on v5
robot = robot.removeFromIgnoredListOfCollisionFilterGroup({'r_uleg'},'l_uleg');
robot = robot.removeFromIgnoredListOfCollisionFilterGroup({'l_uleg'},'r_uleg');
robot = compile(robot);

v = robot.constructVisualizer;

%% Construct the data_struct


data_struct = struct('contacts',{},'seed',{});
counter =1;
seed_counter = 1;
% kneeling
data_struct(counter).contacts =  {'l_toe','l_knee','r_toe','r_knee'};
data_struct(counter).seed = data.q_sol(:,seed_counter);
counter = counter + 1;


% kneeling + hand on ground
seed_counter = seed_counter + 1;
data_struct(counter).contacts =  {'l_toe','l_knee','r_toe','r_knee','l_hand'};
data_struct(counter).seed= data.q_sol(:,seed_counter);
counter = counter + 1;

% One knee + one hand
seed_counter = seed_counter + 2;
data_struct(counter).contacts =  {'l_toe','l_knee','l_hand'};
data_struct(counter).seed = data.q_sol(:,seed_counter);
counter = counter + 1;

% One knee + One hand, pose 2
seed_counter = seed_counter + 1;
data_struct(counter).contacts =  {'l_toe','l_knee','l_hand'};
data_struct(counter).seed = data.q_sol(:,seed_counter);
counter = counter+1;


% One knee, right foot down
seed_counter = seed_counter + 1;
data_struct(counter).contacts =  {'l_toe','l_knee','l_hand','r_foot'};
data_struct(counter).seed = data.q_sol(:,seed_counter);
counter = counter+1;


% One knee + no hands
seed_counter = seed_counter + 2;
data_struct(counter).contacts = {'l_toe','l_knee','r_foot'};
data_struct(counter).seed = data.q_sol(:,seed_counter);
counter = counter+1;


%% KinematicPoseTrajectory
kpt = KinematicPoseTrajectory(robot,data_struct);
kpt.setSolverOptions('snopt','superbasicslimit',3000);

% this takes about 860 seconds
tic;
[q_sol,exitflag,infeasible_constraint_name, kpt] = kpt.solve();
exitflag
infeasible_constraint_name
toc;
qtraj = kpt.constructTrajectory(q_sol);

%% Playback with minimal contact model robot

atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_minimal_contact.urdf'];

options.floating = true;
robot = RigidBodyManipulator(atlas_urdf,options);
robot = PoseKinematics.addVisualContactPts(robot);
robot = compile(robot);
v = robot.constructVisualizer;
qtraj = qtraj.setOutputFrame(robot.getPositionFrame);
v.playback(qtraj,struct('slider',true));

%% IK post-processing
[q_ik,t_ik] = kpt.inverseKinPointwise_2(q_sol,10);

qtraj_ik = PPTrajectory(foh(t_ik,q_ik));
qtraj_ik = qtraj_ik.setOutputFrame(robot.getPositionFrame);
v.playback(qtraj_ik,struct('slider',true));























