function runAtlasStateMachine()

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

init_controller = SilentInitController('init',r);
manip_controller = AtlasManipController('manip',r);
state_machine = DRCStateMachine(struct(manip_controller.name,manip_controller,...
  init_controller.name,init_controller),init_controller.name);

state_machine.run();

end
