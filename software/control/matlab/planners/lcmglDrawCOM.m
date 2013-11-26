contact_threshold = 50;
lc = lcm.lcm.LCM.getSingleton();
lcmgl = drake.util.BotLCMGLClient(lc,'COM');
ground_monitor = drake.util.MessageMonitor(bot_core.pose_t(),'utime');
lc.subscribe('POSE_GROUND',ground_monitor);

force_torque_frame = AtlasForceTorque();
force_torque_frame.subscribe('EST_ROBOT_STATE');
l_foot_fz_idx = find(strcmp('l_foot_fz',force_torque_frame.coordinates));
l_foot_tx_idx = find(strcmp('l_foot_tx',force_torque_frame.coordinates));
l_foot_ty_idx = find(strcmp('l_foot_ty',force_torque_frame.coordinates));
r_foot_fz_idx = find(strcmp('r_foot_fz',force_torque_frame.coordinates));
r_foot_tx_idx = find(strcmp('r_foot_tx',force_torque_frame.coordinates));
r_foot_ty_idx = find(strcmp('r_foot_ty',force_torque_frame.coordinates));

options.floating = true;
options.dt = 0.001;
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
nq = atlas.getNumDOF();
l_foot = atlas.findLinkInd('l_foot');
r_foot = atlas.findLinkInd('r_foot');

atlas_state_frame = atlas.getStateFrame();
atlas_state_frame.subscribe('EST_ROBOT_STATE');

getModelFlag = false;
model_listener = RobotModelListener('ROBOT_MODEL');

display('Waiting for robot model ...');
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
ground_z = 0.05;
l_foot_in_contact = false;
r_foot_in_contact = false;
l_foot_pt = [0;0;0];
r_foot_pt = [0;0;0];
fz_l = 0;
fz_r = 0;
while true
  msg = ground_monitor.getNextMessage(0);
  if ~isempty(msg)
    ground_pose = bot_core.pose_t(msg);
    ground_z = ground_pose.pos(3)+0.05;
  end
  [x,ts] = getNextMessage(atlas_state_frame,0);
  if ~isempty(x)
    kinsol = doKinematics(robot,x(1:nq));
    com = getCOM(robot,kinsol);
    com(3) = ground_z+0.05;
    lcmgl.glColor3f(1,0,0);
    lcmgl.sphere(com,0.02,20,20);
%     if l_foot_in_contact || r_foot_in_contact
%       if l_foot_in_contact
        l_foot_pt_in_world = forwardKin(robot,kinsol,l_foot,l_foot_pt);
        l_foot_pt_in_world(3) = ground_z+0.05;
%         lcmgl.glColor3f(1,0,1);
%         lcmgl.sphere(l_foot_pt_in_world,0.02,20,20);
%       else
%         l_foot_pt_in_world = [0;0;0];
%       end
%       if r_foot_in_contact
        r_foot_pt_in_world = forwardKin(robot,kinsol,r_foot,r_foot_pt);
        r_foot_pt_in_world(3) = ground_z+0.05;
%         lcmgl.glColor3f(0,1,1);
%         lcmgl.sphere(r_foot_pt_in_world,0.02,20,20);
%       else
%         r_foot_pt_in_world = [0;0;0];
%       end
%       if l_foot_in_contact || r_foot_in_contact
        cop = (fz_l*l_foot_pt_in_world + fz_r*r_foot_pt_in_world)/(fz_l+fz_r);
        lcmgl.glColor3f(0,0,1);
        lcmgl.sphere(cop,0.02,20,20);
%       end
%     end
    lcmgl.switchBuffers();
  end
  force_torque = getNextMessage(force_torque_frame,0);
  if ~isempty(force_torque)
    % Compute COP
    fz_l = force_torque(l_foot_fz_idx);
    fz_r = force_torque(r_foot_fz_idx);
%     if fz_l > contact_threshold
      l_foot_in_contact = true;
      tx_l = force_torque(l_foot_tx_idx);
      ty_l = force_torque(l_foot_ty_idx);
      l_foot_pt = [-ty_l/fz_l; -tx_l/fz_l; 0];
%     else
%       l_foot_in_contact = false;
%       fz_l = 0;
%     end
%     if fz_r > contact_threshold
      r_foot_in_contact = true;
      tx_r = force_torque(r_foot_tx_idx);
      ty_r = force_torque(r_foot_ty_idx);
      r_foot_pt = [-ty_r/fz_r; -tx_r/fz_r; 0];
%     else
%       r_foot_in_contact = false;
%       fz_r = 0;
%     end
  end
end
