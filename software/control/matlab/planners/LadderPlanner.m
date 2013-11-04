function LadderPlanner
  %NOTEST
  location = 'base';
  status_code = 6;

  % simple quasistatic stepping test for atlas. uses the footstep planner
  % with very slow step speeds to generate a quasistatic stepping plan. for
  % execution it uses approximate IK to publish position references to the
  % robot

  approved_footstep_plan_listener = FootstepPlanListener('APPROVED_FOOTSTEP_PLAN');

  waiting = true;

  addpath(fullfile(getDrakePath,'examples','ZMP'));

  % load robot model
  urdf_filename = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_no_hands.urdf');
  r_collision = Atlas(urdf_filename);
  r = Atlas();
  load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
  r = removeCollisionGroupsExcept(r,{'toe','heel'});
  r = compile(r);
  r = r.setInitialState(xstar);
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');

  lc = lcm.lcm.LCM.getSingleton();
  qnom_mon = drake.util.MessageMonitor(drc.robot_posture_preset_t,'utime');
  lc.subscribe('COMMITTED_POSTURE_PRESET',qnom_mon);
  qnom_state = '';
  fixed_links = [];

  % setup frames
  state_frame = getStateFrame(r);
  state_frame.subscribe('EST_ROBOT_STATE');

  nq = getNumDOF(r);

  x0 = xstar;
  qstar = xstar(1:nq);
  q0 = qstar;

  % create footstep and ZMP trajectories
  footstep_planner = FootstepPlanner(r);
  step_options = footstep_planner.defaults;
  step_options.max_num_steps = 2;
  step_options.min_num_steps = 1;
  step_options.step_speed = 0.005;
  step_options.follow_spline = true;
  step_options.right_foot_lead = true;
  step_options.ignore_terrain = true;
  step_options.nom_step_width = r.nom_step_width;
  step_options.nom_forward_step = r.nom_forward_step;
  step_options.max_forward_step = r.max_forward_step;
  step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;
  step_options.full_foot_pose_constraint = true;
  step_options.step_height = 0.05;
  
  msg =['QS Stepping Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);

  while true
    while waiting
      [x,~] = getNextMessage(state_frame,10);
      if (~isempty(x))
        x0=x;
        q0 = x0(1:nq);
      end

      [footsteps, ~] = approved_footstep_plan_listener.getNextMessage(10);
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
        tol = 0.01;
        if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_LFTHND_FIX)
          fixed_links = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',[0;0.1;0],'tolerance',tol);
          % with fixed joint hands, the link name is huge.
          %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('l_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX)
          fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',[0;0.1;0],'tolerance',tol);
          %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('r_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
          fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',[0;0.1;0],'tolerance',tol);
          fixed_links(2) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',[0;0.1;0],'tolerance',tol);
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

    [support_times,support, comtraj, foottraj, ~, ~] = walkingPlanFromSteps(r, x0, footsteps,step_options);
    link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

    [x_data,ts] = robotLadderPlan(r,r_collision, q0, qstar, comtraj.tspan(end), link_constraints,support_times,support);

    plan_pub.publish(ts,x_data);

    waiting = true;
  end
end
