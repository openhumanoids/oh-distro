%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

lcm_mon = DrillTaskLCMMonitor(atlas);
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'drill_path_history');

%% get affordance fits
[wall,drill] = lcm_mon.getWallAndDrillAffordances();
while isempty(wall) || isempty(drill)
  [wall,drill] = lcm_mon.getWallAndDrillAffordances();
end

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
  
  if norm(drill_pts(:,1) - point_last) > .002;
    points(:,buffer_ind) = drill_pts(:);
    
    if buffer_length < buffer_size
      buffer_length = buffer_length + 1;
    end
    
    if buffer_ind == buffer_size
      buffer_ind = 1;
    else
      buffer_ind = buffer_ind + 1;
    end
    
    
    lcmgl.glColor3f(0,1,0); %green
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    %redraw buffer
    for i=1:buffer_length,
      j = buffer_ind - i;
      if j <= 0
        j = j + buffer_size;
      end
      if i~=1
        lcmgl.glVertex3d(points(1,j),points(2,j),points(3,j));
      end
      lcmgl.glVertex3d(points(1,j),points(2,j),points(3,j));
    end
    lcmgl.glEnd();
    lcmgl.switchBuffers();
  end
  point_last = drill_pts(:,1);
  pause(.05);
end