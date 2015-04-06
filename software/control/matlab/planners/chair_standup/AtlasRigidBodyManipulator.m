function obj = AtlasRigidBodyManipulator(urdf)

  clear options;
  options.floating = true;
  obj = RigidBodyManipulator(urdf,options);

  hand_position_right = [0; -0.195; -0.01];
  hand_position_left = hand_position_right;
  hand_orientation_right = [0; -3.1415/2; 3.1415];
  hand_orientation_left = [0; 3.1415/2; 3.1415];
  
  options.hand_right = 'robotiq_weight_only';
  options.hand_left = 'robotiq_weight_only';


  if strcmp(options.hand_right,'robotiq_weight_only')
    options_hand.weld_to_link = findLinkId(obj,'r_hand');
    obj = obj.addRobotFromURDF([getenv('DRC_BASE'),'/software/drake/examples/Atlas/urdf/robotiq_box.urdf'], hand_position_right, hand_orientation_right, options_hand);
  end

  if strcmp(options.hand_left,'robotiq_weight_only')
    options_hand.weld_to_link = findLinkId(obj,'l_hand');
    obj = obj.addRobotFromURDF([getenv('DRC_BASE'),'/software/drake/examples/Atlas/urdf/robotiq_box.urdf'], hand_position_left,hand_orientation_left, options_hand);
  end


end