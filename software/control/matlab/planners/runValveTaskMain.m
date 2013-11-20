%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
%% get affordance fits


useVisualization = true;
publishPlans = true;
useRightHand = false;
allowPelvisHeight = false;
lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

finger_pt_on_hand = zeros(3,1);
finger_axis_on_hand = [1; 0; 0];

%%
while true 
  display('Waiting for control message...')
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();
  
%   ctrl_type = drc.drill_control_t.RQ_VALVE_CIRCLE_PLAN;
%   ctrl_data = .2;

  switch ctrl_type
    case drc.drill_control_t.RQ_VALVE_CIRCLE_PLAN
      if sizecheck(ctrl_data, [1 1])
        display('Received request for circle plan, waiting for valve affordance.');
        % block until we've gotten a valve affordance
        valve = lcm_mon.getValveAffordance();
%         valve.center = [100; -.5; .5];
%         valve.normal = [1;0;0];
        while isempty(valve)
          valve = lcm_mon.getValveAffordance();
        end
        drill_pub = drillPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand,...
          valve.normal, useRightHand, useVisualization, publishPlans, allowPelvisHeight);

        arc = ctrl_data(1);
%         speed = ctrl_data(2);
        x_finger_center = valve.center;
        
        display('Got valve affordance, waiting for state estimate.');
        q0 = lcm_mon.getStateEstimate();
        display('Generating circular plan');
        [xtraj_circ,snopt_info_circ,infeasible_constraint_circ] = drill_pub.createCircularPlan(q0, x_finger_center, arc, .05);
        display('Received circular plan');
      else
        display('Invalid size of control data. Expected 1x1');
        send_status(4,0,0,'Invalid size of control data. Expected 1x1');
      end
  end
  
  pause(.1);
end