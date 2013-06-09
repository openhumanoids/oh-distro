function runSeatedDrivingStateMachine(options)

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
options.floating = false;  %turn this to true to do the muddy seat driving
% NOTE: floating = true does not work with harness controller, 
% something wierd with the neck and torso.
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

harness_controller = HarnessController('seated_driving',r,Inf);

controllers = struct(harness_controller.name,harness_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);

% robot_options.floating = true;
% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),robot_options);
% 
% robot_with_car = RigidBodyManipulator();
% robot_with_car = addRobotFromURDF(robot_with_car,strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),[0;0;0],[0;0;0],robot_options);
% robot_with_car = addRobotFromURDF(robot_with_car,strcat(getenv('DRC_PATH'),'/models/mit_gazebo_objects/mit_vehicle/model_drake.urdf'),[0;0;0],[0;0;0],struct('floating',false));
% robot_with_car = TimeSteppingRigidBodyManipulator(robot_with_car,0.001,robot_options);
% robot_with_car = compile(robot_with_car);

% options.multi_robot = robot_with_car;
% if(~isfield(options,'use_mex')) options.use_mex = false; end
% if(~isfield(options,'debug')) options.debug = true; end

% driving_controller = SeatedDrivingController('seated_driving',r,options);

% controllers = struct(driving_controller.name,driving_controller);

% state_machine = DRCStateMachine(controllers,driving_controller.name);
state_machine.run();

end


