function runAtlasStateMachine(controller_type)

if nargin < 1
  controller_type = 1; % 1: PID, 2: PID+manip params, 3: PD+gravity comp, 4: inverse dynamics
end

options.floating = true;
% options.floating = false;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

options.controller_type = controller_type;
init_controller = SilentInitController('init',r);
manip_controller = AtlasManipController('manip',r,options);
state_machine = DRCStateMachine(struct(manip_controller.name,manip_controller,...
  init_controller.name,init_controller),init_controller.name);

state_machine.run();

end
