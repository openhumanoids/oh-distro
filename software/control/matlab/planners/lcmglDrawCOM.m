lc = lcm.lcm.LCM.getSingleton();
lcmgl = drake.util.BotLCMGLClient(lc,'COM');
ground_monitor = drake.util.MessageMonitor(bot_core.pose_t(),'utime');
lc.subscribe('POSE_GROUND',ground_monitor);

options.floating = true;
options.dt = 0.001;
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
nq = atlas.getNumDOF();

atlas_state_frame = atlas.getStateFrame();
atlas_state_frame.subscribe('EST_ROBOT_STATE');

getModelFlag = false;
model_listener = RobotModelListener('ROBOT_MODEL');
while(~getModelFlag)
  data = model_listener.getNextMessage(5);
  if(~isempty(data))
    getModelFlag = true;
    l_hand_mode = data.left_hand_mode;
    r_hand_mode = data.right_hand_mode;
    if(l_hand_mode == 0)
      l_hand_str = 'no hand';
    elseif(l_hand_mode == 1)
      l_hand_str = 'sandia hand';
    elseif(l_hand_mode == 2)
      l_hand_str = 'irobot hand';
    end
    if(r_hand_mode == 0)
      r_hand_str = 'no hand';
    elseif(r_hand_mode == 1)
      r_hand_str = 'sandia hand';
    elseif(r_hand_mode == 2)
      r_hand_str = 'irobot hand';
    end
    send_status(4,0,0,sprintf('receive model with left %s, right %s\n',l_hand_str,r_hand_str));
  end
end
if(l_hand_mode == 0 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RN.urdf'),options);
elseif(l_hand_mode == 0 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RS.urdf'),options);
elseif(l_hand_mode == 0 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RI.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LS_RN.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LS_RI.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RN.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RS.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RI.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 3)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RI_Hose.urdf'),options);
else
  error('The urdf for the model does not exist');
end

display('Plotting the projection of the COM onto the ground via LCMGL');
have_ground_pose = false;
while true
  [x,ts] = getNextMessage(atlas_state_frame,0);
  if ~isempty(x)
    kinsol = doKinematics(robot,x(1:nq));
    if have_ground_pose
      com = getCOM(robot,kinsol);
      com(3) = ground_pose.pos(3)+0.05;
      lcmgl.glColor3f(1,0,0);
      lcmgl.sphere(com,0.02,20,20);
      lcmgl.switchBuffers();
    end
  end
  msg = ground_monitor.getNextMessage(0);
  if ~isempty(msg)
    ground_pose = bot_core.pose_t(msg);
    have_ground_pose = true;
  end
end
