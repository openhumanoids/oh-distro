%NOTEST
%%
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
%% get affordance fits


useVisualization = false;
doAutoCommit = true;
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
        useRightHand, useVisualization, publishPlans, doAutoCommit);
display('Got affordance, getting nominal steering poses')
[q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
steering_depth = 0;
%%
while true 
%   display('Waiting for control message...')
%   driving_ctrl = lcm_mon.getDrivingControlMsg();
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();
  
  switch ctrl_type
    case drc.drill_control_t.DRIVING_CONTROL
      if sizecheck(ctrl_data, [2 1])
        steering_angle = ctrl_data(1)*2*pi;
        ankle_angle = ctrl_data(2);
        steering_speed = .5;
        ankle_speed = .2;
        q0 = lcm_mon.getStateEstimate();
        xtraj = planner.createDrivingPlan(q0, steering_angle, ankle_angle, steering_speed, ankle_speed, steering_vec, q_vec);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
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
        useRightHand, useVisualization, publishPlans, doAutoCommit);
      display('Got affordance, getting nominal steering poses')
      [q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
    case drc.drill_control_t.SET_STEERING_DEPTH
      if sizecheck(ctrl_data, [1 1])
        display(sprintf('Setting steering depth to %f, getting nominal steering poses',ctrl_data(1)));
        steering_depth = ctrl_data(1);
        
        planner = drivingPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand, ...
          root_body, steer_center_in_root + steering_depth*steer_axis_in_root,...
          steer_axis_in_root, steer_zero_vec_in_root,steer_radius, ...
          useRightHand, useVisualization, publishPlans, doAutoCommit);
        display('Got affordance, getting nominal steering poses')
        [q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
      else
        send_status(4,0,0,'Invalid size of control data. Expected 3x1');
      end
  end
end