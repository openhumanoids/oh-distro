function runCrawlingPlanner(location,options)
FORWARD = 1;
WALK = 0;
ZMP_WALK = 1;
ZMP_TROT = 2;
if nargin < 1; location = 'base'; end
if nargin < 2; options = struct(); end
if ~isfield(options,'draw'); options.draw = false; end
if ~isfield(options,'faceup'); options.faceup = true; end
if ~isfield(options,'delta_yaw'); options.delta_yaw = 10*180/pi; end


if strcmp(location, 'base')
  status_code = 6;
else
  status_code = 7;
end

%addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));
addpath(strcat(getenv('DRC_PATH'),'/control/matlab/grasp_execution'));


options.ignore_terrain = true;
options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

if options.faceup
  r = removeCollisionGroupsExcept(r,{'heel','knuckle'});
else
  r = removeCollisionGroupsExcept(r,{'toe','knuckle'});
end

r = setTerrain(r,DRCTerrainMap(false,struct('name','Crawl Plan','status_code',status_code,'fill', true,'normal_radius',2)));
%r = setTerrain(r,DRCFlatTerrainMap());
r = compile(r);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
nq = getNumDOF(r);
actuated = getActuatedJoints(r);

body_spec.body_ind = findLinkInd(r,'pelvis');
body_spec.pt = zeros(3,1);

if options.faceup
  foot_spec(1).body_ind = findLinkInd(r,'l_hand');
  foot_spec(2).body_ind = findLinkInd(r,'r_hand');
  foot_spec(3).body_ind = findLinkInd(r,'r_foot');
  foot_spec(4).body_ind = findLinkInd(r,'l_foot');

  [~,foot_spec(1).contact_pt_ind] = getContactPoints(findLink(r,'l_hand'),'knuckle');
  [~,foot_spec(2).contact_pt_ind] = getContactPoints(findLink(r,'r_hand'),'knuckle');
  foot_spec(3).contact_pt_ind = 1;
  foot_spec(4).contact_pt_ind = 2;
  d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/suppine_crawl2.mat'));
  qstar = d.x0(1:nq);
  x0 = d.x0;
else
  foot_spec(1).body_ind = findLinkInd(r,'l_hand');
  foot_spec(2).body_ind = findLinkInd(r,'r_hand');
  foot_spec(3).body_ind = findLinkInd(r,'l_foot');
  foot_spec(4).body_ind = findLinkInd(r,'r_foot');

  [~,foot_spec(1).contact_pt_ind] = getContactPoints(findLink(r,'r_hand'),'knuckle');
  [~,foot_spec(2).contact_pt_ind] = getContactPoints(findLink(r,'l_hand'),'knuckle');
  [~,foot_spec(3).contact_pt_ind] = getContactPoints(findLink(r,'l_foot'),'toe_mid');
  [~,foot_spec(4).contact_pt_ind] = getContactPoints(findLink(r,'r_foot'),'toe_mid');
  d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bear_crawling.mat'));
  qstar = d.x_nom(1:nq);
  x0 = d.x_nom;
end
options.x_nom = x0;

lc = lcm.lcm.LCM.getSingleton();

committed_goal_mon = drake.util.MessageMonitor(drc.walking_goal_t(),'utime');
lc.subscribe('COMMITTED_CRAWLING_GOAL',committed_goal_mon);

goal_mon = drake.util.MessageMonitor(drc.walking_goal_t(),'utime');
lc.subscribe('CRAWLING_GOAL',goal_mon);

while true
  msg =['Crawl Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
  waiting = true;
  committed = false;

  while waiting
    [x,~] = getNextMessage(state_frame,10);
    if (~isempty(x))
      x0=x;
    end
    data = committed_goal_mon.getNextMessage(10);
    if (~isempty(data))
      goal = drc.walking_goal_t(data);
      msg = ['Crawl Plan (', location, '): committed plan received']; disp(msg); send_status(status_code,0,0,msg);
      waiting = false;
      committed = true;
    else
      data = goal_mon.getNextMessage(10);
      if (~isempty(data))
        goal = drc.walking_goal_t(data);
        if (goal.crawling)
          msg =['Crawl Plan (', location, '): plan received']; disp(msg); send_status(status_code,0,0,msg);
          waiting = false;
          committed = false;
        end
      end
    end
  end
  if (~isempty(data))
    %options.step_speed = goal.step_speed;
    %options.step_height = goal.step_height;
    %options.step_length = goal.nom_forward_step;
    %options.ignore_terrain = goal.ignore_terrain;
    %options.max_num_steps = goal.max_num_steps;
    %options.min_num_steps = goal.min_num_steps;
    target_xy = [goal.goal_pos.translation.x;goal.goal_pos.translation.y];

    [firstTurn, forwardSegment] = turnThenCrawl(target_xy,x0,options);
    options.gait = ZMP_TROT;
    options.num_steps = forwardSegment.num_steps;

    % Plan first turn
    options.direction = forwardSegment.direction;
    display('Getting qtraj ...');
    [qtraj{1},support_times{1},supports{1},V{1},comtraj{1},zmptraj{1},link_constraints{1}] = crawlingPlan(r,x0,body_spec,foot_spec,options)

    % Plan forward crawling
    %options.direction = FORWARD;
   %[support_times{2},supports{2},V{2},comtraj{2},zmptraj{2},qtraj{2}] = ...
    %crawlingPlan(r,[eval(qtraj{1},qtraj{1}.tspan(2)); zeros(nq,1)],body_spec,foot_spec,options)

    % Plan second turn
    %options.direction = secondTurn.direction;
    %[support_times{3},supports{3},V{3},comtraj{3},zmptraj{3},qtraj{3}] = ...
    %crawlingPlan(r,[eval(qtraj{2},qtraj{2}.tspan(2)); zeros(nq,1)],body_spec,foot_spec,options)


    support_times_full = cell2mat(support_times);
    supports_full = [supports{:}];

    s1_full = V{1}.s1;
    %s1_full = V{1}.s1.append(V{2}.s1.shiftTime(V{1}.s1.tspan(2)));
    %s1_full = s1_full.append(V{3}.s1.shiftTime(s1_full.tspan(2)));

    s2_full = V{1}.s2;
    %s2_full = V{1}.s2.append(V{2}.s2.shiftTime(V{1}.s2.tspan(2)));
    %s2_full = s2_full.append(V{3}.s2.shiftTime(s2_full.tspan(2)));

    comtraj_full = comtraj{1};
    %comtraj_full = comtraj{1}.append(comtraj{2}.shiftTime(comtraj{1}.tspan(2)));
    %comtraj_full = comtraj_full.append(comtraj{3}.shiftTime(comtraj_full.tspan(2)));

    zmptraj_full = zmptraj{1};
    %zmptraj_full = zmptraj{1}.append(zmptraj{2}.shiftTime(zmptraj{1}.tspan(2)));
    %zmptraj_full = zmptraj_full.append(zmptraj{3}.shiftTime(zmptraj_full.tspan(2)));

    % Assemble full plan
    qtraj_full = qtraj{1};
    %qtraj_full = qtraj{1}.append(qtraj{2}.shiftTime(qtraj{1}.tspan(2)));
    %qtraj_full = qtraj_full.append(qtraj{3}.shiftTime(qtraj_full.tspan(2)));
    %qtraj_full = cell2mat(qtraj);
    xtraj = [qtraj_full; 0*qtraj_full];
    %xtraj = x0;
    ts = 0:0.025:support_times_full(end)-eps; %TODO: Get real ts
    x_data = eval(xtraj,ts);
  end
  %ts = 0;
  %
  if committed
    mu = goal.mu;
    %options.ignore_terrain = goal.ignore_terrain;
    crawling_plan = struct('S',V{1}.S,'s1',s1_full,'s2',s2_full,...
      'support_times',support_times_full,'supports',{supports_full},'comtraj',comtraj_full(1:2),'qtraj',qtraj_full(actuated),'mu',mu,...
      'link_constraints',link_constraints{1},'zmptraj',zmptraj_full,'qnom',qstar,'ignore_terrain',options.ignore_terrain)
    msg =['Crawl Plan (', location, '): Publishing committed plan...']; disp(msg); send_status(status_code,0,0,msg);
    makeFist;
    walking_pub = WalkingPlanPublisher('WALKING_PLAN');
    walking_pub.publish(0,crawling_plan);
  else
    msg =['Crawl Plan (', location, '): Publishing robot plan...']; disp(msg); send_status(status_code,0,0,msg);
    joint_names = r.getStateFrame.coordinates(1:nq);
    plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_CRAWLING_PLAN');
    plan_pub.publish(ts,x_data);
  end

end

end

function [turn, forwardSegment] = turnThenCrawl(target_xy, x0, options)
if ~isfield(options,'min_num_steps') options.min_num_steps = 20; end
if ~isfield(options,'max_num_steps') options.max_num_steps = 20; end
if ~isfield(options,'num_steps') options.num_steps = 20; end
if ~isfield(options,'duty_factor') options.duty_factor = 2/3; end
if ~isfield(options,'step_length') options.step_length = .3; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'com_height') options.com_height = .35; end
if ~isfield(options,'comfortable_footpos') options.comfortable_footpos = [-.7 -.7 .6 .6; .3 -.3 -.3 .3]; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'gait') options.gait = 2; end
if ~isfield(options,'draw') options.draw = true; end
if ~isfield(options,'debug') options.debug = false; end
if ~isfield(options,'x_nom') options.x_nom = x0; end
if ~isfield(options,'delta_yaw') options.delta_yaw = 10*pi/180; end
  fieldcheck(options,'step_length');

  delta_xy = target_xy - x0(1:2);
  target_heading = atan2(delta_xy(2),delta_xy(1));
  z_proj  = rpy2rotmat(x0(4:6))*[0;0;1];
  current_heading = atan2(z_proj(2),z_proj(1));

  delta_heading = target_heading - current_heading;
  target_distance = norm(delta_xy);

  if options.draw
    sfigure(7); clf; axis equal;hold on;
    plot(x0(1),x0(2),'bo',target_xy(1),target_xy(2),'gs');
    line([0 cos(current_heading)],[0 sin(current_heading)],'Color','b');
    line(target_distance*[0 cos(target_heading)],target_distance*[0 sin(target_heading)],'Color','g');
  end

  turn.direction = sign(delta_heading);
  turn.num_steps = 4*ceil(delta_heading/options.delta_yaw);
  forwardSegment.direction = 0;
  forwardSegment.num_steps = ceil(target_distance/options.step_length);
  forwardSegment.num_steps = max(min(forwardSegment.num_steps,options.max_num_steps),options.min_num_steps);
end
