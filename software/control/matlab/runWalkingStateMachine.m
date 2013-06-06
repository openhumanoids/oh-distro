function runWalkingStateMachine(options)

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.namesuffix = '';
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
%r = setTerrain(r,DRCTerrainMap(true,struct('name','WalkingStateMachine','fill',true)));
r = compile(r);

if(~isfield(options,'use_mex')) options.use_mex = false; end
if(~isfield(options,'use_hand_ft')) options.use_hand_ft = false; end
if(~isfield(options,'debug')) options.debug = false; end

standing_controller = StandingController('standing',r,options);
walking_controller = WalkingController('walking',r,options);
bracing_controller = BracingController('bracing',r,options);

controllers = struct(standing_controller.name,standing_controller, ...
                      walking_controller.name,walking_controller,...     
                      bracing_controller.name,bracing_controller);
                    
state_machine = DRCStateMachine(controllers,standing_controller.name);

state_machine.run();

end


