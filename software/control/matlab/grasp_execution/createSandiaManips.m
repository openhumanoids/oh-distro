function [r_left,r_right] = createSandiaManips()
  dt=0.001;
  options.floating=false; % If this is set to true there are issues, need to fix them later
  drc_path=getenv('DRC_PATH');

    %filelocation=[drc_path 'models/mit_gazebo_models/mit_robot_hands/floatingbase_urdfs/sandia_hand_drake_left_floating.urdf'];
    filelocation=[drc_path '/models/mit_gazebo_models/mit_robot_hands/drake_urdfs/sandia_hand_drake_left.urdf'];
    r_left=TimeSteppingRigidBodyManipulator(filelocation,dt,options);

    %filelocation=[drc_path 'models/mit_gazebo_models/mit_robot_hands/floatingbase_urdfs/sandia_hand_drake_right_floating.urdf'];
    filelocation=[drc_path '/models/mit_gazebo_models/mit_robot_hands/drake_urdfs/sandia_hand_drake_right.urdf'];
    r_right=TimeSteppingRigidBodyManipulator(filelocation,dt,options);

end
