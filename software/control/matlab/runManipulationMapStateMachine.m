function runManipulationMapStateMachine()%or runPreComputedPoseGraphServer()
  % listens to COMMITTED_MANIP_MAP, AFF_GOAL,AFF_STATE and EST_ROBOT_STATE
  % and generates a COMMITTED_ROBOT_PLAN msg.
  
  addpath(fullfile(pwd,'frames'));

  % load atlas model
  options.floating = true;
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
  r = removeCollisionGroupsExcept(r,{'heel','toe'});
  r = compile(r);

  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); 

  state_machine = DRCManipMapStateMachine(r);

  % atlas state subscriber
  state_frame = r.getStateFrame();
  state_frame.subscribe('EST_ROBOT_STATE');

  map_listener = AffIndexedRobotPlanListener('COMMITTED_MANIP_MAP',true);
  planviz_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
  plan_pub = RobotPlanPublisher('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
  affgoal_listener = AffGoalListener('DRIVING_MANIP_CMD');
  driving_aff_status_pub = DrivingAffordanceStatusPublisher();%'DRIVING_STEERING_AFFORDANCE_STATUS');
  
  t0=clock;
  elapsed_ms=0;
  ts0 = clock;
  elapsed_ms2 = 0;
  ts = tic;
  ts0 = tic;
  while(1)
      elapsed_ms = toc(ts)*1000;
      elapsed_ms2 = toc(ts0)*1000;
      if(elapsed_ms>100) %10Hz state update should be sufficient
          [x,ts] = getNextMessage(state_frame,0);
          if (~isempty(x))
              float_offset=6;
              qcurrent=x(1:getNumDOF(r));
              state_machine.setActiveState(qcurrent);
          end
          ts = tic;
      end
      
      % Publish driving manipulation affordance status
      if (elapsed_ms2>1000)% && state_machine.manip_map_received)
          publishDrivingActuationStatus (state_machine,driving_aff_status_pub);
          ts0 = tic;
      end;
      
      [xmap,affinds] = map_listener.getNextMessage(0);
      if (~isempty(xmap))
          disp('candidate manipulation plan was committed');
          state_machine.setMap(xmap,affinds);
          % or setGraph
      end
      
      goal = affgoal_listener.getNextMessage(0);
      if (~isempty(goal))
          disp('candidate manipulation plan was rejected');
          q_breaks = state_machine.getPlanGivenAffGoal(goal);
          qdot_breaks = 0*q_breaks;
          s_breaks=linspace(0,1,size(q_breaks,2));
          t_breaks=s_breaks.*(length(s_breaks)*0.001);
          planviz_pub.publish(t_breaks,[q_breaks;qdot_breaks]);
          plan_pub.publish(t_breaks,[q_breaks;qdot_breaks]);
      end
  end
  
end



%
%
