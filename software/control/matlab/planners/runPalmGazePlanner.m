%%
display('Constructing atlas objects');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
rbmoptions.floating = true;
rbmoptions.visual = false;

r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);
%%
display('Waiting for model...')
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
    elseif(l_hand_mode == 4)
      l_hand_str = 'robotiq hand';      
    end
    if(r_hand_mode == 0)
      r_hand_str = 'no hand';
    elseif(r_hand_mode == 1)
      r_hand_str = 'sandia hand';
    elseif(r_hand_mode == 2 || r_hand_mode == 3)
      r_hand_str = 'irobot hand';
    elseif(r_hand_mode == 4)
      r_hand_str = 'robotiq hand';      
    end
    send_status(4,0,0,sprintf('receive model with left %s, right %s\n',l_hand_str,r_hand_str));
  end
end

if l_hand_mode == 4
  T_palm_hand_l = inv(HT([0;0.19016;0.015],0, -2.618, 0));
else
  T_palm_hand_l = inv(HT([0;0.11516;0.015],pi/2,pi,pi));
end
if r_hand_mode == 4
  T_palm_hand_r = inv(HT([0;-0.19016;-0.015],0,pi,pi));
else
  T_palm_hand_r = inv(HT([0;-0.11516;-0.015],pi/2,0,0));
end
%%
lcm_mon = palmGazeLCMMonitor(atlas);

doVisualization = false;
doPublish = true;
allowPelvisHeight = false;    
 
planner = palmGazePlanner(r,atlas, T_palm_hand_l, T_palm_hand_r, doVisualization, doPublish, allowPelvisHeight);

speed = .05;

display('Ready to receive requests');
while(true)
  q0 = lcm_mon.getStateEstimate();

  left_palm_pose = lcm_mon.getLeftPalmPose();
  if ~isempty(left_palm_pose)
    [xtraj,snopt_info,infeasible_constraint] = planner.createGazePlan(q0, left_palm_pose, speed, false);
  end
  
  right_palm_pose = lcm_mon.getRightPalmPose();
  if ~isempty(right_palm_pose)
    [xtraj,snopt_info,infeasible_constraint] = planner.createGazePlan(q0, right_palm_pose, speed, true);
  end

  pause(.01);
end
