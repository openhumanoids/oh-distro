% The main program for the drill task

%% Setup our simulink objects and lcm monitor
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

%% Wait for drill and wall affordance and create planner
publishPlans = true;
useRightHand = true;
useVisualization = false;
allowPelvisHeight = true;

lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

disp('waiting for drill and wall affordances...');

[wall,drill] = lcm_mon.getWallAndDrillAffordances();
while isempty(wall) || isempty(drill)
  [wall,drill] = lcm_mon.getWallAndDrillAffordances();
end

drill.guard_pos = [    0.15
   -0.2602



    0.0306];
  drill.drill_axis = [1;0;0];
finger_pt_on_hand = [0; 0.2752; 0.015];
finger_axis_on_hand = [0;1;0];

button_pub = drillButtonPlanner(r,atlas,drill.button_pos, drill.button_normal, drill.drill_axis,...
 finger_pt_on_hand, finger_axis_on_hand, useRightHand, useVisualization, publishPlans);

drill_pub = drillPlanner(r,atlas,drill.guard_pos, drill.drill_axis,...
  wall.normal, useRightHand, useVisualization, publishPlans, allowPelvisHeight);
drill_points = [wall.targets wall.targets(:,1)];


% drilling state machine initialization
segment_index = 1;
cut_lengths = sum((drill_points(:,2:end) - drill_points(:,1:end-1)).*(drill_points(:,2:end) - drill_points(:,1:end-1)));
[~,diagonal_index] = max(cut_lengths);

short_cut = .03;
long_cut = .1;
target_radius = .05;
predrill_distance = .1;

xtraj_nominal = []; %to know if we've got a plan
xtraj_button = []; %to know if we've got a plan

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
  end
  
  disp('waiting for control message...');
  
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();

  
  switch ctrl_type
    case drc.drill_control_t.REFIT_DRILL
        
      disp('updating drill affordance');

      new_drill = lcm_mon.getDrillAffordance();
      if ~isempty(new_drill)
        drill = new_drill;
        drill_pub = drill_pub.updateDrill(drill.guard_pos, drill.drill_axis);
        button_pub = button_pub.updateDrill(drill.button_pos, drill.button_normal, drill.drill_axis);
      else
        send_status(4,0,0,'Cannot update drill, no affordance found');
      end
      
    case drc.drill_control_t.RQ_NOMINAL_PLAN
      %hand picked joints that make for a decent guess
      q0_init = [zeros(6,1); 0.0355; 0.0037; 0.0055; zeros(12,1); -1.2589; 0.3940; 2.3311; -1.8152; 1.6828; zeros(6,1); -0.9071;0];
      
      q0 = lcm_mon.getStateEstimate();
      q0_init(setdiff(1:r.num_q,[1; 2; 6; drill_pub.joint_indices])) = q0(setdiff(1:r.num_q,[1; 2; 6; drill_pub.joint_indices]));
      
      target_centroid = mean(wall.targets,2);
      
      % create wall coordinate frame
      wall_z = [0;0;1];
      wall_z = wall_z - wall_z'*wall.normal*wall.normal;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z, wall.normal);
      
      q0_init(1:3) = target_centroid - wall.normal*.7 - .5*wall_z + .3*wall_y;
      q0_init(6) = atan2(wall.normal(2), wall.normal(1));
      [xtraj_nominal,snopt_info_nominal,infeasible_constraint_nominal] = drill_pub.findDrillingMotion(q0_init, drill_points, true, 0);
      
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
        
        [xtraj_arm_init,snopt_info_arm_init,infeasible_constraint_arm_init] = drill_pub.createInitialReachPlan(q0, drill_f - predrill_distance*wall.normal, 5);
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
      x_drill_reach = wall.targets(:,1) - predrill_distance*wall.normal;
      
      [xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = drill_pub.createInitialReachPlan(q0, x_drill_reach, 5);
    case drc.drill_control_t.RQ_DRILL_IN_PLAN
      q0 = lcm_mon.getStateEstimate();
      [xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, wall.targets(:,1), 5);
      
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
      else
      % create wall coordinate frame
      wall_z = [0;0;1];
      wall_z = wall_z - wall_z'*wall.normal*wall.normal;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z, wall.normal);
      
        cut_length = long_cut;
      end
      
      segment_dir = (drill_points(:,segment_index+1) -drill_points(:,segment_index));
      segment_dir = segment_dir/norm(segment_dir);
      
      line_param = -(drill_points(:,segment_index) - drill0)'*(drill_points(:,segment_index+1) - drill_points(:,segment_index))/norm(drill_points(:,segment_index+1)-drill_points(:,segment_index))^2;
      
      nearest_point = drill_points(:,segment_index) + line_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));
      
      dist_to_cut = norm(drill0 - nearest_point);
      if dist_to_cut < .07
        cut_param = min(1,cut_length/cut_lengths(segment_index) + line_param);
        drill_target = drill_points(:,segment_index) + cut_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));
      else
        drill_target = nearest_point;
      end
      
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
    case drc.drill_control_t.RQ_BUTTON_PREPOSE_PLAN
      q0 = lcm_mon.getStateEstimate();
      last_button_offset = [-.1;0;0];
      [xtraj_button,snopt_info_button,infeasible_constraint_button] = button_pub.createPrePokePlan(q0, 5);
    case drc.drill_control_t.RQ_BUTTON_DELTA_PLAN
      if sizecheck(ctrl_data, [3 1])
        
        if ~isempty(xtraj_button)
          xlast = xtraj_button.eval(xtraj_button.tspan(2));
          q0 = lcm_mon.getStateEstimate();
          q0(setdiff(1:34',button_pub.button_joint_indices)) = xlast(setdiff(1:34',button_pub.button_joint_indices));
          button_offset = last_button_offset + ctrl_data(1:3);
%           last_button_offset = button_offset;
          [xtraj_button,snopt_info_button,infeasible_constraint_button] = button_pub.createPokePlan(q0, button_offset, 5);
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