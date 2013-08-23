function runAtlasStateMachine()

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

manip_controller = AtlasManipController('manip',r);
state_machine = DRCStateMachine(struct(manip_controller.name,manip_controller),manip_controller.name);

state_machine.run();

end
