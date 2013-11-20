%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

lcm_mon = drillTaskLCMMonitor(atlas, true);
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'drill_path_history');
line_buffer = drc.control.LCMGLLineBuffer(lcmgl,0,1,0);
%% get affordance fits
[wall,drill] = lcm_mon.getWallAndDrillAffordances();
while isempty(wall) || isempty(drill)
  [wall,drill] = lcm_mon.getWallAndDrillAffordances();
end
drill.guard_pos = [    0.25
   -0.2602



    0.0306];
  drill.drill_axis = [1;0;0];

%%
hand_body = 29;
buffer_size = 1000;
buffer_ind = 1;
buffer_length = 0;
points = zeros(3,buffer_size);
point_last = inf(3,1);
while(true)
  q = lcm_mon.getStateEstimate();
  kinsol = r.doKinematics(q);
  drill_pts = r.forwardKin(kinsol,hand_body,drill.guard_pos);
  
  if norm(drill_pts(:,1) - point_last) > .002
    line_buffer.addPointAndDisplay(drill_pts(1), drill_pts(2), drill_pts(3));
    point_last = drill_pts(:,1);
  end
  
  pause(.05);
end