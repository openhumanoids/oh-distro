function LadderPlanner(options)
  if nargin < 1, options = struct(); end;
  if ~isfield(options,'stability_type'), options.stability_type = 'tension'; end;
  if ~isfield(options,'verbose'), options.stability_type = 'false'; end;
  
  %NOTEST
  status_code = 6;
  
  approved_footstep_plan_listener = FootstepPlanListener('APPROVED_FOOTSTEP_PLAN');
  
  waiting = true;
  
  hand_tol = 0.0;
  
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
  l_foot = r.findLinkInd('l_foot');
  r_foot = r.findLinkInd('r_foot');
  
  x0 = xstar;
  qstar = xstar(1:nq);
  q0 = qstar;

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

  switch options.stability_type
    case 'quasistatic'
      ladder_opts.fine.use_quasistatic_constraint =  true;
      ladder_opts.fine.use_arm_tension_constraint =  false;
      ladder_opts.fine.use_com_constraint = true;
      ladder_opts.fine.use_incr_com_constraint =     false;
      ladder_opts.fine.use_utorso_constraint =       true;
      ladder_opts.fine.com_tol = 0.0;
    case 'tension'
      ladder_opts.fine.use_quasistatic_constraint =  false;
      ladder_opts.fine.use_arm_tension_constraint =  false;
      ladder_opts.fine.use_com_constraint = true;
      ladder_opts.fine.use_incr_com_constraint =     false;
      ladder_opts.fine.use_utorso_constraint =       true;
      ladder_opts.fine.com_tol = 0.005;
  end
  ladder_opts.fine.use_final_com_constraint = true;
  ladder_opts.fine.use_arm_constraints =         true;
  ladder_opts.fine.use_total_arm_constraints =   false;
  ladder_opts.fine.use_pelvis_gaze_constraint =  true;
  ladder_opts.fine.use_pelvis_constraint =       true;
  ladder_opts.fine.use_knee_constraint =         true;
  ladder_opts.fine.use_ankle_constraint =        true;
  ladder_opts.fine.use_neck_constraint =         true;
  ladder_opts.fine.use_collision_constraint =    false;
  ladder_opts.fine.use_smoothing_constraint =    false;
  ladder_opts.fine.use_swing_foot_euler_constraint = true;
  ladder_opts.fine.smooth_output = true;
  ladder_opts.fine.smoothing_span = 5;
  ladder_opts.fine.smoothing_method = 'moving'; 
  ladder_opts.fine.n = 1;
  ladder_opts.fine.compute_intro = true;
  ladder_opts.fine.shrink_factor = 0.5;
  ladder_opts.fine.utorso_threshold = 10*pi/180;
  ladder_opts.fine.pelvis_gaze_threshold = 10*pi/180;
  ladder_opts.fine.ankle_limit = 15*pi/180;
  ladder_opts.fine.knee_lb = 35*pi/180*ones(2,1);
  ladder_opts.fine.knee_ub = inf*pi/180*ones(2,1);
  ladder_opts.fine.hand_threshold = sin(1*pi/180);
  ladder_opts.fine.hand_cone_threshold = sin(1*pi/180);
  ladder_opts.fine.hand_pos_tol = 0.0;
  ladder_opts.fine.pelvis_threshold = 0.05;
  ladder_opts.fine.com_incr_tol = 0.02;
  ladder_opts.fine.com_tol_max = 0.5;
  ladder_opts.fine.qs_margin = 0.0;
  ladder_opts.fine.arm_tol = 5*pi/180;
  ladder_opts.fine.arm_tol_total = 30*pi/180;
  ladder_opts.fine.verbose = options.verbose;

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
        l_hand_str = 'hook hand';
        l_hand_offset = [0;0.219+0.125;-0.092];
        l_hand_axis = [1;0;0];
        ladder_opts.fine.hand_threshold = sin(0.5*pi/180);
        ladder_opts.fine.shrink_factor = 1.3;
        ladder_opts.fine.final_shrink_factor = 0.2;
      elseif(l_hand_mode == 1)
        l_hand_str = 'sandia hand';
        l_hand_offset = [0.025;0.25;0.04];
        l_hand_axis = [-1;1;0];
      elseif(l_hand_mode == 2)
        l_hand_str = 'irobot hand';
        l_hand_offset = [0;0.15;0];
        l_hand_axis = [1;0;0];
        ladder_opts.fine.final_shrink_factor = 1.2;
        %         l_hand_axis = [0;0;1];
      end
      if(r_hand_mode == 0)
        r_hand_str = 'hook hand';
        r_hand_offset = [0;-0.219-0.125;-0.092];
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

  msg =['Ladder Plan: Listening for plans']; disp(msg); send_status(status_code,0,0,msg);


  while true
    while waiting
      pause(0.1);
      [x,~] = getNextMessage(robot_state_coder,10);
      if (~isempty(x))
        x0=x;
        q0 = x0(1:nq);
      end

      [footsteps, step_options] = approved_footstep_plan_listener.getNextMessage(10);
      if (~isempty(footsteps))
        fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',r_hand_offset,'tolerance',hand_tol);
        fixed_links(2) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',l_hand_offset,'tolerance',hand_tol);
        msg =['Ladder Plan: Footstep plan received']; disp(msg); send_status(status_code,0,0,msg);
        waiting = false;
      end

      qnom_data = qnom_mon.getNextMessage(0);

      if(~isempty(qnom_data))
        send_status(status_code,0,0,'Ladder Plan: Got new committed posture preset');
        qnom_msg = drc.robot_posture_preset_t(qnom_data);
        if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_LFTHND_FIX || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX || qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
          qnom_state = 'current';
        end
        if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_LFTHND_FIX)
          qnom_state = 'fix_left';
          fixed_links = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',l_hand_offset,'tolerance',hand_tol);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX)
          qnom_state = 'fix_right';
          fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',r_hand_offset,'tolerance',hand_tol);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
          qnom_state = 'fix_both';
          fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',r_hand_offset,'tolerance',hand_tol);
          fixed_links(2) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',l_hand_offset,'tolerance',hand_tol);
        else
          fixed_links = [];
        end
      end

      if(strcmp(qnom_state,'current'))
        % update xstar to current posture.
        q0 = x0(1:nq);
      end

    end

    [support_times,support, comtraj, foottraj, ~, ~] = walkingPlanFromSteps(r, x0, footsteps,step_options);
    link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

    ee_info = struct();
    ee_info.feet(1) = link_constraints([link_constraints.link_ndx] == l_foot);
    ee_info.feet(2) = link_constraints([link_constraints.link_ndx] == r_foot);
    ee_info.feet(1).idx = l_foot;
    ee_info.feet(2).idx = r_foot;
    ee_info.hands(1) = link_constraints([link_constraints.link_ndx] == l_hand);
    ee_info.hands(2) = link_constraints([link_constraints.link_ndx] == r_hand);
    ee_info.hands(1).idx = l_hand;
    ee_info.hands(2).idx = r_hand;
    ee_info.hands(1).axis = l_hand_axis;
    ee_info.hands(1).axis = r_hand_axis;

    ee_info.feet(1).support_traj = supportTraj(l_foot,support_times,support);
    ee_info.feet(2).support_traj = supportTraj(r_foot,support_times,support);

    ladder_opts.fine.comtraj = comtraj;
    [x_data,ts] = robotLadderPlanLeanBack(r, q0, q0, comtraj, ee_info, support_times,ladder_opts);


    msg =['Ladder Plan: MAKE SURE THE BOT IS IN USER MODE']; disp(msg); send_status(status_code,0,0,msg);
    plan_pub.publish(ts,x_data);

    waiting = true;
    msg =['Ladder Plan: Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
  end
end
