%NOTEST
%%
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
%% get affordance fits


useVisualization = false;
publishPlans = true;
useRightHand = false;
allowPelvisHeight = false;
lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);
root_body = 2;
display('Waiting for wheel affordance');

wheel = lcm_mon.getValveAffordance();
while isempty(wheel)
  wheel = lcm_mon.getValveAffordance();
end

q0 = lcm_mon.getStateEstimate();
kinsol = r.doKinematics(q0);
x_root = r.forwardKin(kinsol,root_body,zeros(3,1),2);
R_root = quat2rotmat(x_root(4:7));
x_root = x_root(1:3);

steer_center_in_root = R_root'*(wheel.center - x_root);
steer_axis_in_root = R_root'*wheel.normal;
steer_zero_vec_in_root = R_root'*(wheel.init_pt - wheel.center);
steer_zero_vec_in_root = steer_zero_vec_in_root/norm(steer_zero_vec_in_root);
steer_radius = wheel.radius;

finger_pt_on_hand = [0;.3;0];
finger_axis_on_hand = [0; 1; 0];

planner = drivingPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand, ...
        root_body, steer_center_in_root, steer_axis_in_root, steer_zero_vec_in_root,steer_radius, ...
        useRightHand, useVisualization, publishPlans);
display('Got affordance, getting nominal steering poses')
[q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
steering_depth = 0;
leg_zero_angles = [];
%%
while true 
%   display('Waiting for control message...')
%   driving_ctrl = lcm_mon.getDrivingControlMsg();
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();
  
  switch ctrl_type
    case drc.drill_control_t.DRIVING_PULSE
      if isempty(leg_zero_angles)
        send_status(4,0,0,'Leg zero angles must be set first');
      elseif sizecheck(ctrl_data, [3 1])
        ankle_position = ctrl_data(1);
        duration = ctrl_data(2);
        doAutoCommit = ctrl_data(3); 
        q0 = lcm_mon.getStateEstimate();
        q0(planner.leg_joint_indices) = leg_zero_angles;
        xtraj = planner.createDrivingPulse(q0, ankle_position, duration, doAutoCommit);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
    case drc.drill_control_t.SET_STEERING_ANGLE
      if sizecheck(ctrl_data, [2 1])
        steering_angle = ctrl_data(1)*2*pi;
        doAutoCommit = ctrl_data(2); 
        steering_speed = .5;
        q0 = lcm_mon.getStateEstimate();
        if ~isempty(leg_zero_angles)
          q0(planner.leg_joint_indices) = leg_zero_angles;
        end
        xtraj = planner.createSteeringPlan(q0, steering_angle, steering_speed, steering_vec, q_vec, doAutoCommit);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 2x1');
      end
    case drc.drill_control_t.REFIT_STEERING
      display('Refitting steering wheel, waiting for affordance');
      wheel = lcm_mon.getValveAffordance();
      while isempty(wheel)
        wheel = lcm_mon.getValveAffordance();
      end
      q0 = lcm_mon.getStateEstimate();
      kinsol = r.doKinematics(q0);
      x_root = r.forwardKin(kinsol,root_body,zeros(3,1),2);
      R_root = quat2rotmat(x_root(4:7));
      x_root = x_root(1:3);
      
      steer_center_in_root = R_root'*(wheel.center - x_root);
      steer_axis_in_root = R_root'*wheel.normal;
      steer_zero_vec_in_root = R_root'*(wheel.init_pt - wheel.center);
      steer_zero_vec_in_root = steer_zero_vec_in_root/norm(steer_zero_vec_in_root);
      steer_radius = wheel.radius;
      
      planner = drivingPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand, ...
        root_body, steer_center_in_root + steering_depth*steer_axis_in_root,...
        steer_axis_in_root, steer_zero_vec_in_root,steer_radius, ...
        useRightHand, useVisualization, publishPlans);
      display('Got affordance, getting nominal steering poses')
      [q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
    case drc.drill_control_t.LEFT_LEG_JOINT_TELEOP
      if sizecheck(ctrl_data, [7 1])
        q0 = lcm_mon.getStateEstimate();
        leg_angles = ctrl_data(1:6);
        doAutoCommit = ctrl_data(7);
        speed = .1;
        xtraj = planner.createLegTeleopPlan(q0, leg_angles, speed, doAutoCommit);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 7x1');
      end
    case drc.drill_control_t.SET_DRIVING_ZERO_POSITION
      if sizecheck(ctrl_data, [6 1])
        leg_zero_angles = ctrl_data(1:6);
        send_status(4,0,0,'Set leg zero position');
      else
        send_status(4,0,0,'Invalid size of control data. Expected 6x1');
      end
    case drc.drill_control_t.SET_STEERING_DEPTH
      if sizecheck(ctrl_data, [1 1])
        display(sprintf('Setting steering depth to %f, getting nominal steering poses',ctrl_data(1)));
        steering_depth = ctrl_data(1);
        
        planner = drivingPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand, ...
          root_body, steer_center_in_root + steering_depth*steer_axis_in_root,...
          steer_axis_in_root, steer_zero_vec_in_root,steer_radius, ...
          useRightHand, useVisualization, publishPlans);
        display('Got affordance, getting nominal steering poses')
        [q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
  end
end