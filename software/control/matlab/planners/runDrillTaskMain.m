%% The main program for the drill task
publishPlans = true;
useRightHand = false;
useVisualization = false;
allowPelvisHeight = true;


%% Setup our simulink objects and lcm monitor
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
rbmoptions.floating = true;
if ~useVisualization
  rbmoptions.visual = false;
end
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);

drill_depth = 0;

%% Wait for drill and wall affordance and create planner

lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

disp('waiting for drill and wall affordances...');

[wall,drill] = lcm_mon.getWallAndDrillAffordances();
while isempty(wall) || isempty(drill)
  [wall,drill] = lcm_mon.getWallAndDrillAffordances();
end
% 
% drill.guard_pos = [    0.15
%    -0.2602
%     0.0306];
%   drill.drill_axis = [1;0;0];
if useRightHand
    drill.drill_axis = -[.5;0;sqrt(3)/2];  % or -.5, and -.04 on x below
    drill.guard_pos = [-.07;-.3;-.15];
    finger_pt_on_hand = [0; 0.2752; 0.015];
    finger_axis_on_hand = [0;1;0];
else
    drill.drill_axis = -[.5;0;sqrt(3)/2];  % or -.5, and -.04 on x below
    drill.guard_pos = [-.07;.3;-.15];
    finger_pt_on_hand = [0; -0.2752; 0.015];
    finger_axis_on_hand = [0;-1;0];
end


button_pub = drillButtonPlanner(r,atlas,drill.button_pos, drill.button_normal, drill.drill_axis,...
 finger_pt_on_hand, finger_axis_on_hand, useRightHand, useVisualization, publishPlans);

drill_pub = drillPlanner(r,atlas,drill.guard_pos, drill.drill_axis,...
  wall.normal, useRightHand, useVisualization, publishPlans, allowPelvisHeight);
drill_points = [wall.targets wall.targets(:,1)];
drill_points = drill_points + repmat(drill_depth*wall.normal,1,size(drill_points,2));


% drilling state machine initialization
segment_index = 1;
cut_lengths = sum((drill_points(:,2:end) - drill_points(:,1:end-1)).*(drill_points(:,2:end) - drill_points(:,1:end-1)));
[~,diagonal_index] = max(cut_lengths);

short_cut = .05;
long_cut = .05;
target_radius = .04;
predrill_distance = .12;

xtraj_nominal = []; %to know if we've got a plan
xtraj_button = []; %to know if we've got a plan
%%
while(true)
  % Update wall
  new_wall = lcm_mon.getWallAffordance();
  if ~isempty(new_wall)
    if ~isequal(new_wall.normal, wall.normal) || ~isequal(new_wall.targets, wall.targets)
      wall = new_wall;
      
      disp('updating drill wall targets from new wall affordance');

    end
        
    drill_pub = drill_pub.updateWallNormal(wall.normal);
    drill_points = [wall.targets wall.targets(:,1)];
    drill_points = drill_points + repmat(drill_depth*wall.normal,1,size(drill_points,2));
    
    cut_lengths = sum((drill_points(:,2:end) - drill_points(:,1:end-1)).*(drill_points(:,2:end) - drill_points(:,1:end-1)));
    [~,diagonal_index] = max(cut_lengths);
  end
  
  disp('waiting for control message...');
  
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();

  
  switch ctrl_type
    case drc.drill_control_t.RQ_GOTO_BUTTON_PRESET
      disp('Searching for a button hand posture');
      if useRightHand
        gaze_in_pelvis = [-.8;1;.1];
      else
        gaze_in_pelvis = [-.8;-1;.1];
      end
      
      gaze_in_pelvis = gaze_in_pelvis/norm(gaze_in_pelvis);
      q0 = lcm_mon.getStateEstimate();
      kinsol = r.doKinematics(q0);
      pelvis_pose = r.forwardKin(kinsol,2,zeros(3,1),2);
      R = quat2rotmat(pelvis_pose(4:7));
      button_gaze = R*gaze_in_pelvis;
      
      button_pos_min = [.5;0;-inf];
      button_pos_max = [inf;.2;inf];
      pelvis_frame = [R pelvis_pose(1:3); 0 0 0 1];
      [xtraj,snopt_info,infeasible_constraint] = button_pub.createButtonPreposePlan(q0, button_gaze, 5*pi/180, pelvis_frame, button_pos_min, button_pos_max, 5);

    case drc.drill_control_t.REFIT_DRILL
        
      disp('updating drill affordance');

      new_drill = lcm_mon.getDrillAffordance();
      if ~isempty(new_drill)
        drill = new_drill;
        disp(sprintf('new drill axis is %3.3f, %3.3f, %3.3f', drill.drill_axis(1), drill.drill_axis(2), drill.drill_axis(3)));
        drill_pub = drill_pub.updateDrill(drill.guard_pos, drill.drill_axis);
        button_pub = button_pub.updateDrill(drill.button_pos, drill.button_normal, drill.drill_axis);
      else
        send_status(4,0,0,'Cannot update drill, no affordance found');
      end
      
    case drc.drill_control_t.RQ_NOMINAL_PLAN
      %hand picked joints that make for a decent guess
      q0_init = [zeros(6,1); 0.0355; 0.0037; 0.0055; zeros(12,1); -1.2589; 0.3940; 2.3311; -1.8152; 1.6828; zeros(6,1); -0.9071;0];
      q0_init(drill_pub.joint_indices)=[0.6315; 0.0867; 0.1009; -0.5439; 0.6321;1.3733; -0.8194; 0.9061; -1.0353];
%       q0_init(drill_pub.joint_indices) = randn(9,1);
      q0 = lcm_mon.getStateEstimate();
      q0_init(setdiff(1:r.num_q,[1; 2; 6; drill_pub.joint_indices])) = q0(setdiff(1:r.num_q,[1; 2; 6; drill_pub.joint_indices]));
      
      target_centroid = mean(drill_points(:,1:end-1),2);
      
      % create wall coordinate frame
      wall_z = [0;0;1];
      wall_z = wall_z - wall_z'*wall.normal*wall.normal;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z, wall.normal);
      
      depth_increase = .05;
      target_expansion = .08;
      drill_points_expanded = drill_points;
      for i=1:size(drill_points_expanded,2),
        drill_points_expanded(:,i) = drill_points_expanded(:,i) + target_expansion*(drill_points_expanded(:,i) - target_centroid)/norm(drill_points_expanded(:,i) - target_centroid) + wall.normal*depth_increase;
      end
      
      q0_init(1:3) = target_centroid - wall.normal*.7 - .2*wall_z + 0.0*wall_y;
      q0_init(6) = atan2(wall.normal(2), wall.normal(1)) + .3;
      [xtraj_nominal,snopt_info_nominal,infeasible_constraint_nominal] = drill_pub.findDrillingMotion(q0_init, drill_points_expanded, true, 0);
      
    case drc.drill_control_t.RQ_WALKING_GOAL
      if ~isempty(xtraj_nominal)
        disp('sending walking goal');
        x_end = xtraj_nominal.eval(0);
        pose = [x_end(1:3); rpy2quat(x_end(4:6))];
        drill_pub.publishWalkingGoal(pose);
      else
        send_status(4,0,0,'Nominal trajectory not instantiated yet, cannot create a walking goal');
      end
      
    case drc.drill_control_t.RQ_ARM_PREPOSE_PLAN
      if ~isempty(xtraj_nominal)
        q0 = lcm_mon.getStateEstimate();
     
        qf = xtraj_nominal.eval(0);
        qf = qf(1:34);
        posture_index = setdiff((1:r.num_q)',[drill_pub.joint_indices]');
        qf(posture_index) = q0(posture_index);
        kinsol = r.doKinematics(qf);
        drill_f = r.forwardKin(kinsol,drill_pub.hand_body,drill_pub.drill_pt_on_hand);
        
        
        [xtraj_arm_init,snopt_info_arm_init,infeasible_constraint_arm_init] = drill_pub.createInitialReachPlan(q0, drill_f - (drill_depth + predrill_distance)*wall.normal, 10, qf);
      else
        send_status(4,0,0,'Nominal trajectory not instantiated yet, cannot create a walking goal');
      end
      
    case drc.drill_control_t.RQ_NOMINAL_FIXED_PLAN
              disp('getting state estimate');
      q0 = lcm_mon.getStateEstimate();
            disp('computing fixed nominal plan');

      [xtraj_nominal,snopt_info_nominal,infeasible_constraint_nominal] = drill_pub.findDrillingMotion(q0, drill_points, false);
      
    case drc.drill_control_t.RQ_PREDRILL_PLAN
      segment_index = 1; % RESETS THE SEGMENT INDEX!
      q0 = lcm_mon.getStateEstimate();
      x_drill_reach = drill_points(:,1) - predrill_distance*wall.normal;
      
      qseed = q0;
      if ~isempty(xtraj_nominal)
        x_nom_end = xtraj_nominal.eval(xtraj_nominal.tspan(2));
        qseed(drill_pub.joint_indices) =  x_nom_end(drill_pub.joint_indices);
      end
      
      [xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = drill_pub.createInitialReachPlan(q0, x_drill_reach, 5, qseed);
    case drc.drill_control_t.RQ_DRILL_IN_PLAN
      q0 = lcm_mon.getStateEstimate();
      [xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_points(:,1), 5);
      
    case drc.drill_control_t.RQ_NEXT_DRILL_PLAN
      q0 = lcm_mon.getStateEstimate();
        
      kinsol = r.doKinematics(q0);
      drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill.guard_pos);
      
      in_goal = norm(drill0 - drill_points(:,segment_index+1)) < target_radius;
      
      if in_goal
        segment_index = segment_index + 1;
        if segment_index == diagonal_index,
          cut_length = short_cut;
        elseif segment_index == size(drill_points,2)
          segment_index = 1; % reset to the beginning, may not be a great idea
        else
          cut_length = long_cut;
        end
        display(sprintf('Hit target, setting segment index to %d',segment_index))
      else
        cut_length = long_cut;
      end
      
      segment_dir = (drill_points(:,segment_index+1) -drill_points(:,segment_index));
      segment_dir = segment_dir/norm(segment_dir);
      
      line_param = -(drill_points(:,segment_index) - drill0)'*(drill_points(:,segment_index+1) - drill_points(:,segment_index))/norm(drill_points(:,segment_index+1)-drill_points(:,segment_index))^2;
      
      nearest_point = drill_points(:,segment_index) + line_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));
      
      dist_to_cut = norm(drill0 - nearest_point);

      if dist_to_cut < cut_length
        cut_length_on_path = sqrt(cut_length^2 - dist_to_cut^2);
        cut_param = min(1,cut_length_on_path/cut_lengths(segment_index) + line_param);
        drill_target = drill_points(:,segment_index) + cut_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));
        display(sprintf('Distance to cut is %f.  Old line param: %f, new line param: %f',dist_to_cut, line_param, cut_param))
      else
        drill_target = nearest_point;
        display(sprintf('Distance to cut is %f.  Old line param: %f, new line param: %f',dist_to_cut, line_param, line_param))
      end
%       keyboard
      [xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_target, 5);
    case drc.drill_control_t.RQ_DRILL_TARGET_PLAN
      if sizecheck(ctrl_data, [3 1])
        drill_target = ctrl_data(1:3);
        q0 = lcm_mon.getStateEstimate();
        kinsol = r.doKinematics(q0);
        drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill.guard_pos);
        [xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_target, 5);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
      
    case drc.drill_control_t.RQ_DRILL_DELTA_PLAN
      % create wall coordinate frame
      wall_z = [0;0;1];
      wall_z = wall_z - wall_z'*wall.normal*wall.normal;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z, wall.normal);
      
      if sizecheck(ctrl_data, [3 1])
        delta = ctrl_data(1:3);
        q0 = lcm_mon.getStateEstimate();
        world_delta = delta(1)*wall.normal + delta(2)*wall_y + delta(3)*wall_z;
        kinsol = r.doKinematics(q0);
        drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill.guard_pos);
        drill_target = drill0 + world_delta;
        [xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_target, 5);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
    case drc.drill_control_t.SET_DRILL_DEPTH
      if sizecheck(ctrl_data, [1 1])
        drill_depth = ctrl_data(1);
        display(sprintf('Setting drill depth to %f',drill_depth));
      else
        send_status(4,0,0,'Invalid size of control data. Expected 1x1');
      end
    case drc.drill_control_t.RQ_BUTTON_PREPOSE_PLAN
      q0 = lcm_mon.getStateEstimate();
      if useRightHand
        last_button_offset = [0;.1;0];
      else
        last_button_offset = [0;-.1;0];
      end
      [xtraj_button,snopt_info_button,infeasible_constraint_button] = button_pub.createPrePokePlan(q0,last_button_offset, 5);
    case drc.drill_control_t.RQ_BUTTON_DELTA_PLAN
      if sizecheck(ctrl_data, [3 1])
        
        if ~isempty(xtraj_button)
          xlast = xtraj_button.eval(xtraj_button.tspan(2));
          q0 = lcm_mon.getStateEstimate();
          q0(setdiff(1:34',[1;2;3;4;5;6;button_pub.finger_joint_indices])) = xlast(setdiff(1:34',[1;2;3;4;5;6;button_pub.finger_joint_indices]));
          button_offset = last_button_offset + ctrl_data(1:3);
%           last_button_offset = button_offset;
          [xtraj_button,snopt_info_button,infeasible_constraint_button] = button_pub.createPokePlan(q0, button_offset, .02);
          %         % use back and off-hand joints from last plan
          %         q0(button_pub.button_joint_indices) = xlast(button_pub.button_joint_indices);
          %         q0(button_pub.back_joint_indices) = xlast(button_pub.back_joint_indices);
        else
          send_status(4,0,0,'Pre-button trajectory not instantiated yet, cannot create a poking plan');
        end
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
      
  end
%   pause();
end
