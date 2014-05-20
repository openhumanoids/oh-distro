function runWalkingPlanner(location, options)

if nargin < 1; location = 'base'; end
if (nargin < 2); options = struct(); end
if ~isfield(options, 'goal_yaw'); options.goal_yaw = 0.0; end
if ~isfield(options, 'goal_y'); options.goal_y = 0.0; end
if ~isfield(options, 'goal_x'); options.goal_x = 0.0; end
if ~isfield(options, 'lcm_plan'); options.lcm_plan = true; end
if ~isfield(options, 'enable_terrainmaps'); options.enable_terrainmaps = true; end

if strcmp(location, 'base')
  status_code = 6;
else
  status_code = 7;
end

debug = true;

addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
options.dt = 0.001;
% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe','inner'});
if options.enable_terrainmaps
  r = setTerrain(r,DRCTerrainMap(false,struct('name',['Walk Plan (', location, ')'],'status_code',status_code,'fill',true)));
end
r = compile(r);
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
nq = getNumDOF(r);
  
lc = lcm.lcm.LCM.getSingleton();
qnom_mon = drake.util.MessageMonitor(drc.robot_posture_preset_t,'utime');
lc.subscribe('COMMITTED_POSTURE_PRESET',qnom_mon);
qnom_state = '';
fixed_links = [];
while true 

  % if nominal posture was not set previously. Load from file.
  if(~strcmp(qnom_state,'current')) 
   d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
   xstar = d.xstar;     
  end
  
  r = r.setInitialState(xstar);
  x0 = xstar; % sho
  qstar = xstar(1:nq);

  pose = [options.goal_x;options.goal_y;0;0;0;options.goal_yaw];
  navgoal = [pose; 20];

  if ~options.lcm_plan
    footsteps = planFootsteps(r, x0, navgoal, struct('plotting', true, 'interactive', false));
  else
    approved_footstep_plan_listener = FootstepPlanListener('APPROVED_FOOTSTEP_PLAN');
    committed_footstep_plan_listener = FootstepPlanListener('COMMITTED_FOOTSTEP_PLAN');
    msg =['Walk Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
    waiting = true;
    committed = false;

    while waiting
      [x,~] = getNextMessage(state_frame,10);
      if (~isempty(x))
        x0=x;
      end
       
      [footsteps, footstep_opts] = committed_footstep_plan_listener.getNextMessage(10);
      if (~isempty(footsteps))
        msg =['Walk Plan (', location, '): committed plan received']; disp(msg); send_status(status_code,0,0,msg);
        waiting = false;
        committed = true;
      else
        [footsteps, footstep_opts] = approved_footstep_plan_listener.getNextMessage(10);
        if (~isempty(footsteps))
          msg =['Walk Plan (', location, '): plan received']; disp(msg); send_status(status_code,0,0,msg);
          waiting = false;
          committed = false;
        end
      end

      if (~isempty(footsteps) && length(footsteps) <= 2)
        msg =['Walk Plan (', location, '): no footsteps in plan']; disp(msg); send_status(status_code,0,0,msg);
        waiting = true;
        committed = false;
      end


      if (~isempty(footsteps))
        if options.enable_terrainmaps
          r.setTerrain(r.getTerrain().setMapMode(footstep_opts.map_command));
        else
          r = setTerrain(r,RigidBodyTerrain());
        end
        
        % Align the first two steps to the current feet poses
        feet_pos = feetPosition(r, x0(1:nq));
        if footsteps(1).is_right_foot
          footsteps(1).pos = feet_pos.right; footsteps(2).pos = feet_pos.left;
        else
          footsteps(1).pos = feet_pos.left; footsteps(2).pos = feet_pos.right;
        end

        % Align the remianing steps to the terrain, or to the walking plane
        if footstep_opts.ignore_terrain
          p0 = feet_pos.right;
          normal = rpy2rotmat(p0(4:6)) * [0;0;1];
          for j = 3:length(footsteps)
            footsteps(j).pos(3) = p0(3) - (1 / normal(3)) * (normal(1) * (footsteps(j).pos(1) - p0(1)) + normal(2) * (footsteps(j).pos(2) - p0(2)));
            footsteps(j).pos = fitPoseToNormal(footsteps(j).pos, normal);
          end
        else
          for j = 3:length(footsteps)
            footsteps(j).pos = fitStepToTerrain(r, footsteps(j).pos, 'orig');
          end
        end

        % Slow down the first and last steps, if necessary
        for j = [length(footsteps)-1,length(footsteps)]
          footsteps(j).step_speed = min([footsteps(j).step_speed, 1.5]);
        end
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
          fixed_links = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
          % with fixed joint hands, the link name is huge.
          %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('l_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_RGTHND_FIX)
          fixed_links = struct('link',r.findLinkInd('r_hand+r_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
          %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('r_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
        elseif (qnom_msg.preset == drc.robot_posture_preset_t.CURRENT_BOTHHNDS_FIX)
          fixed_links = struct('link',r.findLink('r_hand+r_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
          fixed_links(2) = struct('link',r.findLinkInd('l_hand+l_hand_point_mass'),'pt',[0;0.1;0],'tolerance',0.05);
          %fixed_links = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('r_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
          %fixed_links(2) = struct('link',r.findLink(r.getLinkNames{r.findLinkInd('l_foot')+1}),'pt',[0;0.1;0],'tolerance',0.05);
        else
          fixed_links = [];
        end
      end
      
      if(strcmp(qnom_state,'current'))
       % update xstar to current posture.
        qstar = x0(1:nq);
        xstar = x0; 
      end
    end
  end
  mu = footstep_opts.mu;
  [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(r, x0, footsteps, footstep_opts);
  tf = comtraj.tspan(end); assert(abs(eval(V,tf,zeros(4,1)))<1e-4);  % relatively fast check to make sure i'm in the correct frame (x-zmp_tf)
  nq = getNumDOF(r);
  q0 = x0(1:nq);
  link_constraints = buildLinkConstraints(r, q0, foottraj, fixed_links);

  % compute s1,s2 derivatives for controller Vdot computation
  s1dot = fnder(V.s1,1);
  s2dot = fnder(V.s2,1);
  
  if committed
    walking_plan = struct('S',V.S,'s1',V.s1,'s2',V.s2,'s1dot',s1dot,'s2dot',s2dot,...
        'support_times',support_times,'supports',{supports},'comtraj',comtraj,'mu',mu,'t_offset',0,...
        'link_constraints',link_constraints,'zmptraj',zmptraj,'qtraj',qstar,'ignore_terrain',footstep_opts.ignore_terrain)
    msg =['Walk Plan (', location, '): Publishing committed plan...']; disp(msg); send_status(status_code,0,0,msg);
    walking_pub = WalkingPlanPublisher('WALKING_PLAN');
    walking_pub.publish(0,walking_plan);
  else
    [xtraj, ~, ~, ts] = robotWalkingPlan(r, q0, qstar, zmptraj, comtraj, link_constraints);
    % publish robot plan
    msg =['Walk Plan (', location, '): Publishing robot plan...']; disp(msg); send_status(status_code,0,0,msg);
    joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
    joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
    plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');
    plan_pub.publish(ts,xtraj);
    if debug
      tt = 0:0.05:ts(end);
      compoints = zeros(3,length(tt));
      for i=1:length(tt)
        compoints(1:2,i) = comtraj.eval(tt(i));
      end
      compoints(3,:) = getTerrainHeight(r,compoints(1:2,:));
      plot_lcm_points(compoints',[zeros(length(tt),1), ones(length(tt),1), zeros(length(tt),1)],555,'Desired COM',1,true);
    end  
  end

end

end
