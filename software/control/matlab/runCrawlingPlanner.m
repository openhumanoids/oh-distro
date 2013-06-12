function runCrawlingPlanner(location,options)
FORWARD = 1;
WALK = 0;
TROT = 1;
if nargin < 1; location = 'base'; end

if strcmp(location, 'base')
  status_code = 6;
else
  status_code = 7;
end

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));


options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel_mid','knuckle'});
%r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',status_code,'fill', true,'normal_radius',2)));
r = setTerrain(r,DRCFlatTerrainMap());
r = compile(r);
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
nq = getNumDOF(r);

body_spec.body_ind = findLinkInd(r,'pelvis');
body_spec.pt = zeros(3,1);

foot_spec(1).body_ind = findLinkInd(r,'l_hand');
foot_spec(2).body_ind = findLinkInd(r,'r_hand');
foot_spec(3).body_ind = findLinkInd(r,'r_foot');
foot_spec(4).body_ind = findLinkInd(r,'l_foot');

[~,foot_spec(1).contact_pt_ind] = getContactPoints(findLink(r,'l_hand'),'knuckle');
[~,foot_spec(2).contact_pt_ind] = getContactPoints(findLink(r,'r_hand'),'knuckle');
[~,foot_spec(3).contact_pt_ind] = getContactPoints(findLink(r,'r_foot'),'heel_mid');
[~,foot_spec(4).contact_pt_ind] = getContactPoints(findLink(r,'l_foot'),'heel_mid');

d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/suppine_crawl2.mat'));
qstar = d.x0(1:nq);
x0 = d.x0;

lc = lcm.lcm.LCM.getSingleton();

committed_goal_mon = drake.util.MessageMonitor(drc.walking_goal_t(),'utime');
lc.subscribe('COMMITTED_WALKING_GOAL',committed_goal_mon);

goal_mon = drake.util.MessageMonitor(drc.walking_goal_t(),'utime');
lc.subscribe('CRAWLING_NAV_GOAL',goal_mon);

msg =['Crawl Plan (', location, '): Listening for plans']; disp(msg); send_status(status_code,0,0,msg);
waiting = true;
committed = false;

while waiting
  %[x,~] = getNextMessage(state_frame,10);
  %if (~isempty(x))
    %x0=x;
  %end
  data = committed_goal_mon.getNextMessage(10);
  if (~isempty(data))
    goal = drc.walking_goal_t(data);
    msg = ['Crawl Plan (', location, '): committed plan received']; disp(msg); send_status(status_code,0,0,msg);
    waiting = false;
    committed = true;
  else
   data = goal_mon.getNextMessage(10)
    if (~isempty(data))
      goal = drc.walking_goal_t(data);
      if (goal.crawling)
        msg =['Crawl Plan (', location, '): plan received']; disp(msg); send_status(status_code,0,0,msg);
        waiting = false;
        committed = false;
      end
    end
  end
  if (~isempty(data))
    [firstTurn, forwardSegment, secondTurn] = turnCrawlTurnPlan(goal,x0);
    options.gait = WALK;

    % Plan first turn
    options.direction = firstTurn.direction;
    [support_times{1},supports{1},V{1},comtraj{1},zmptraj{1},qtraj{1}] = crawlingPlan(r,x0,body_spec,foot_spec,options)

    % Plan forward crawling
    options.direction = FORWARD;
    [support_times{2},supports{2},V{2},comtraj{2},zmptraj{2},qtraj{2}] = ...
      crawlingPlan(r,[eval(qtraj{1},qtraj{1}.tspan(2)); zeros(nq,1)],body_spec,foot_spec,options)

    % Plan second turn
    options.direction = secondTurn.direction;
    [support_times{3},supports{3},V{3},comtraj{3},zmptraj{3},qtraj{3}] = ...
      crawlingPlan(r,[eval(qtraj{2},qtraj{2}.tspan(2)); zeros(nq,1)],body_spec,foot_spec,options)


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
    qtraj_full = qtraj{1}.append(qtraj{2}.shiftTime(qtraj{1}.tspan(2)));
    qtraj_full = qtraj_full.append(qtraj{3}.shiftTime(qtraj_full.tspan(2)));
    %qtraj_full = cell2mat(qtraj);
    xtraj = [qtraj_full; 0*qtraj_full];
    %xtraj = x0;
    ts = 0:0.025:support_times_full(end)-eps; %TODO: Get real ts
    x_data = eval(xtraj,ts);
    %ts = 0;
    %
    if committed
      mu = 0.5 %TODO: What should mu be?
      crawling_plan = struct('S',V{1}.S,'s1',s1_full,'s2',s2_full,...
        'support_times',support_times_full,'supports',{supports_full},'comtraj',comtraj_full,'qtraj',qtraj_full(7:end,:),'mu',mu,...
        'link_constraints',[],'zmptraj',zmptraj_full,'qnom',qstar,'ignore_terrain',goal.ignore_terrain)
      msg =['Crawl Plan (', location, '): Publishing committed plan...']; disp(msg); send_status(status_code,0,0,msg);
      walking_pub = WalkingPlanPublisher('CRAWLING_PLAN');
      walking_pub.publish(0,crawling_plan);
    else
      msg =['Crawl Plan (', location, '): Publishing robot plan...']; disp(msg); send_status(status_code,0,0,msg);
      joint_names = r.getStateFrame.coordinates(1:nq);
      plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_CRAWLING_PLAN');
      plan_pub.publish(ts,x_data);
    end
  end

end

end

function [firstTurn, forwardSegment, secondTurn] = turnCrawlTurnPlan(goal, x0)
  firstTurn.direction = 1;
  firstTurn.num_steps = 0;
  secondTurn.direction = 1;
  secondTurn.num_steps = 0;
  forwardSegment.num_steps = 0;
  %delta_x = goal.translation.x - x0(1);
  %delta_y = goal.translation.y - x0(2);
  %heading = atan2(delta_y,delta_x);
end
