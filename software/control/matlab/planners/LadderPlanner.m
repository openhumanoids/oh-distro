function LadderPlanner(filename)
  %NOTEST
  location = 'base';
  status_code = 6;
  
  % simple quasistatic stepping test for atlas. uses the footstep planner
  % with very slow step speeds to generate a quasistatic stepping plan. for
  % execution it uses approximate IK to publish position references to the
  % robot
  
  approved_footstep_plan_listener = FootstepPlanListener('APPROVED_FOOTSTEP_PLAN');
  
  waiting = true;
  
  hand_tol = 0.01;
  
  addpath(fullfile(getDrakePath,'examples','ZMP'));
  
  % load robot model
  %urdf_filename = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_no_hands.urdf');
  %r_collision = Atlas(urdf_filename);
  S = warning();
  warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  r = Atlas();
  warning(S);
  load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
  r = removeCollisionGroupsExcept(r,{'toe','heel'});
  r = compile(r);
  r = r.setInitialState(xstar);
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  
  nq = getNumDOF(r);
  
  l_hand = r.findLinkInd('l_hand');
  r_hand = r.findLinkInd('r_hand');
  
  x0 = xstar;
  qstar = xstar(1:nq);
  q0 = qstar;
  
  if nargin > 0
    load(filename);
    
    x0 = [q0;0*q0];
    [support_times,support, comtraj, foottraj, ~, ~] = walkingPlanFromSteps(r, x0, footsteps,step_options);
    link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);
    
    l_hand_constraint_idx = [link_constraints.link_ndx]==l_hand;
    r_hand_constraint_idx = [link_constraints.link_ndx]==r_hand;
    if any(l_hand_constraint_idx)
      link_constraints(l_hand_constraint_idx).axis = l_hand_axis;
    end
    if any(r_hand_constraint_idx)
      link_constraints(r_hand_constraint_idx).axis = r_hand_axis;
    end
    
    [x_data,ts] = robotLadderPlan(r,r, q0, qstar, comtraj, link_constraints,support_times,support);
%     [x_data,ts] = robotLadderPlan(r,r, q0, q0, comtraj, link_constraints,support_times,support);
  else
    plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');
    
    lc = lcm.lcm.LCM.getSingleton();
    qnom_mon = drake.util.MessageMonitor(drc.robot_posture_preset_t,'utime');
    lc.subscribe('COMMITTED_POSTURE_PRESET',qnom_mon);
    qnom_state = '';
    fixed_links = [];
    
    joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
    robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(drc.control.RobotStateCoder(joint_names)));
    robot_state_coder.subscribe('EST_ROBOT_STATE');
    
    % setup frames
    state_frame = getStateFrame(r);
    state_frame.subscribe('EST_ROBOT_STATE');
    % create footstep and ZMP trajectories
    %   footstep_planner = FootstepPlanner(r);
    %   step_options = footstep_planner.defaults;
    %   step_options.max_num_steps = 2;
    %   step_options.min_num_steps = 1;
    %   step_options.step_speed = 0.05;
    %   step_options.follow_spline = true;
    %   step_options.right_foot_lead = true;
    %   step_options.ignore_terrain = true;
    %   step_options.nom_step_width = r.nom_step_width;
    %   step_options.nom_forward_step = r.nom_forward_step;
    %   step_options.max_forward_step = r.max_forward_step;
    %   step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;
    %   step_options.full_foot_pose_constraint = false;
    %   step_options.step_height = 0.05;
    
    % get the robot model first
    % @param l_hand_mode          - 0 no left hand
    %                            - 1 sandia left hand
    %                            - 2 irobot left hand
    % @param r_hand_mode         - 0 no right hand
    %                            - 1 sandia right hand
    %                            - 2 irobot right hand
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
          l_hand_offset = [0;0;0];
          l_hand_axis = [1;0;0];
        elseif(l_hand_mode == 1)
          l_hand_str = 'sandia hand';
          l_hand_offset = [0.025;0.25;0.04];
          l_hand_axis = [-1;1;0];
        elseif(l_hand_mode == 2)
          l_hand_str = 'irobot hand';
          l_hand_offset = [0;0.15;0];
          l_hand_axis = [1;0;0];
          %         l_hand_axis = [0;0;1];
        end
        if(r_hand_mode == 0)
          r_hand_str = 'no hand';
          r_hand_offset = [0;0;0];
          r_hand_axis = [1;0;0];
        elseif(r_hand_mode == 1)
          r_hand_str = 'sandia hand';
          r_hand_offset = [0.025;-0.25;0.04];
          r_hand_axis = [-1;-1;0];
        elseif(r_hand_mode == 2)
          r_hand_str = 'irobot hand';
          r_hand_offset = [0;-0.20;0];
          r_hand_axis = [1;0;0];
          %         r_hand_axis = [0;0;1];
        end
        send_status(4,0,0,sprintf('receive model with left %s, right %s\n',l_hand_str,r_hand_str));
      end
    end
    
    msg =['QS Stepping Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
    
    
    while true
      while waiting
        [x,~] = getNextMessage(robot_state_coder,10);
        if (~isempty(x))
          x0=x;
          q0 = x0(1:nq);
        end
        
        [footsteps, step_options] = approved_footstep_plan_listener.getNextMessage(10);
        if (~isempty(footsteps))
          %footsteps = r.createInitialSteps(x0, navgoal, step_options);
          %for j = 1:length(footsteps)
          %footsteps(j).pos = r.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
          %end
          msg =['QS Stepping Plan (', location, '): plan received']; disp(msg); send_status(status_code,0,0,msg);
          waiting = false;
        end
        
        qnom_data = qnom_mon.getNextMessage(0);
        
        if(~isempty(qnom_data))
          send_status(status_code,0,0,'Got new committed posture preset');
          qnom_msg = drc.robot_posture_preset_t(qnom_data);
          qnom_msg.preset
          if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_LFTHND_FIX || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
            qnom_state = 'current';
          end
          if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_LFTHND_FIX)
            fixed_links = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',l_hand_offset,'tolerance',hand_tol);
            % with fixed joint hands, the link name is huge.
            %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('l_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
          elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX)
            fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',r_hand_offset,'tolerance',hand_tol);
            %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('r_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
          elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
            fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',r_hand_offset,'tolerance',hand_tol);
            fixed_links(2) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',l_hand_offset,'tolerance',hand_tol);
            %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('r_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
            %fixed_links(2) = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('l_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
          else
            fixed_links = [];
          end
        end
        
        if(strcmp(qnom_state,'current'))
          % update xstar to current posture.
          q0 = x0(1:nq);
        end
        
      end
      
      %     for i = 1:length(footsteps)
      %       footsteps(i).step_speed = step_options.step_speed;
      %     end
      [support_times,support, comtraj, foottraj, ~, ~] = walkingPlanFromSteps(r, x0, footsteps,step_options);
      link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);
      
      l_hand_constraint_idx = [link_constraints.link_ndx]==l_hand;
      r_hand_constraint_idx = [link_constraints.link_ndx]==r_hand;
      if any(l_hand_constraint_idx)
        link_constraints(l_hand_constraint_idx).axis = l_hand_axis;
      end
      if any(r_hand_constraint_idx)
        link_constraints(r_hand_constraint_idx).axis = r_hand_axis;
      end
      
      [x_data,ts] = robotLadderPlanLeanBack(r,r, q0, q0, comtraj, link_constraints,support_times,support);
      
      plan_pub.publish(ts,x_data);
      
      waiting = true;
      msg =['QS Stepping Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
    end
  end
end
