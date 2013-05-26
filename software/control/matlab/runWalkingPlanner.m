function runWalkingPlanner(location, lcm_plan, goal_x, goal_y, goal_yaw)
if nargin < 5; goal_yaw = 0.0; end
if nargin < 4; goal_y = 0.0; end
if nargin < 3; goal_x = 2.0; end
if nargin < 2; lcm_plan = true; end
if nargin < 1; location = 'base'; end

debug = true;

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
options.dt = 0.001;
% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap(false,struct('name','Walk Plan','fill',true)));
r = compile(r);

lc = lcm.lcm.LCM.getSingleton();
qnom_mon = drake.util.MessageMonitor(drc.robot_posture_preset_t,'utime');
lc.subscribe('UPDATE_NOMINAL_POSTURE',qnom_mon);
qnom_state = '';
while true 
  
  d = load('data/atlas_fp.mat');
  xstar = d.xstar;
  r = r.setInitialState(xstar);
  state_frame = getStateFrame(r);
  state_frame.subscribe('EST_ROBOT_STATE');

  nq = getNumDOF(r);
  x0 = xstar;
  qstar = xstar(1:nq);

  pose = [goal_x;goal_y;0;0;0;goal_yaw];
  navgoal = [pose; 20];

  if ~lcm_plan
    footsteps = planFootsteps(r, x0, navgoal, struct('plotting', true, 'interactive', false));
  else
    approved_footstep_plan_listener = FootstepPlanListener('APPROVED_FOOTSTEP_PLAN');
    committed_footstep_plan_listener = FootstepPlanListener('COMMITTED_FOOTSTEP_PLAN');
    msg =['Walk Plan (', location, '): Listening for plans']; disp(msg); send_status(6,0,0,msg);
    waiting = true;
    committed = false;
    foottraj = [];

    while waiting
      [x,~] = getNextMessage(state_frame,10);
      if (~isempty(x))
        x0=x;
      end
       
      footsteps = committed_footstep_plan_listener.getNextMessage(10);
      if (~isempty(footsteps))
        msg =['Walk Plan (', location, '): committed plan received']; disp(msg); send_status(6,0,0,msg);
        waiting = false;
        committed = true;
      else
        footsteps = approved_footstep_plan_listener.getNextMessage(10);
        if (~isempty(footsteps))
          msg =['Walk Plan (', location, '): plan received']; disp(msg); send_status(6,0,0,msg);
          waiting = false;
          committed = false;
        end
      end
      
      qnom_data = qnom_mon.getNextMessage(0);
      
      if(~isempty(qnom_data))
        qnom_msg = drc.robot_posture_preset_t(qnom_data);
        if(qnom_msg.preset == drc.robot_posture_preset_t.CURRENT)
          qnom_state = 'current';
        end
      end
      
      if(strcmp(qnom_state,'current'))
        qstar = x0(1:nq);
      end
    end
  end
  [xtraj, qtraj, htraj, supptraj, comtraj, lfoottraj,rfoottraj, V, ts,zmptraj] = walkingPlanFromSteps(r, x0, qstar, footsteps);
  hddot = fnder(htraj,2);
  walking_plan = struct('S',V.S,'s1',V.s1,'htraj',htraj,'hddtraj', ...
      hddot,'supptraj',supptraj,'comtraj',comtraj,'qtraj',[],...
      'lfoottraj',lfoottraj,'rfoottraj',rfoottraj,'zmptraj',zmptraj,'qnom',qstar)
  last_approved_footsteps = footsteps;

  
  % publish robot plan
  msg =['Walk Plan (', location, '): Publishing robot plan...']; disp(msg); send_status(6,0,0,msg);
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
  plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
  plan_pub.publish(ts,xtraj);

  if committed
    msg =['Walk Plan (', location, '): Publishing committed plan...']; disp(msg); send_status(6,0,0,msg);
    walking_pub = WalkingPlanPublisher('WALKING_PLAN');
    walking_pub.publish(0,walking_plan);
  end


  % if 0 % do proper time-varying linear system approach
  %   disp('Computing ZMP controller...');
  %   limp = LinearInvertedPendulum(htraj);
  %   [~,V] = ZMPtracker(limp,zmptraj); 
  % end


  if debug
    tt = 0:0.04:ts(end);
    compoints = zeros(3,length(tt));
    for i=1:length(tt)
      compoints(1:2,i) = comtraj.eval(tt(i));
    end
    compoints(3,:) = getTerrainHeight(r,compoints(1:2,:));
    plot_lcm_points(compoints',[zeros(length(tt),1), ones(length(tt),1), zeros(length(tt),1)],555,'Desired COM',1,true);
  end  

  % msg ='Walk Plan : Waiting for confirmation...'; disp(msg); send_status(6,0,0,msg);
  % plan_listener = RobotPlanListener('COMMITTED_ROBOT_PLAN',true);
  % reject_listener = RobotPlanListener('REJECTED_ROBOT_PLAN',true);
  % waiting = true;
  % execute = true;
  % while waiting
  %   rplan = plan_listener.getNextMessage(100);
  %   if (~isempty(rplan))
  %     % for now don't do anything with it, just use it as a flag
  %     msg ='Walk Plan : Confirmed. Executing...'; disp(msg); send_status(6,0,0,msg);
  %     waiting = false;
  %   end
  %   rplan = reject_listener.getNextMessage(100);
  %   if (~isempty(rplan))
  %     % for now don't do anything with it, just use it as a flag
  %     disp('Walk Plan : Plan rejected.');
  %     waiting = false;
  %     execute = false;
  %   else 
  %   %   plan_pub.publish(ts,xtraj);
  %     pause(0.5);
  %   end
  % end

  % if execute
  %   walking_pub = WalkingPlanPublisher('WALKING_PLAN');
  %   walking_pub.publish(0,struct('S',V.S,'s1',V.s1,'htraj',htraj,'hddtraj', ...
  %     hddot,'supptraj',supptraj,'comtraj',comtraj,'qtraj',[],...
  %     'lfoottraj',lfoottraj,'rfoottraj',rfoottraj,'zmptraj',zmptraj));
  % end
end

end
