function atlasQuasistaticStepping
%NOTEST

% simple quasistatic stepping test for atlas. uses the footstep planner
% with very slow step speeds to generate a quasistatic stepping plan. for
% execution it uses approximate IK to publish position references to the
% robot

navgoal = [1;0;0;0;0;0];
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
ref_frame = AtlasPositionRef(r);

nq = getNumDOF(r);

x0 = xstar;
q0 = x0(1:nq);

% create footstep and ZMP trajectories
footstep_planner = FootstepPlanner(r);
step_options = footstep_planner.defaults;
step_options.max_num_steps = 2;
step_options.min_num_steps = 1;
step_options.step_speed = 0.05;
step_options.follow_spline = true;
step_options.right_foot_lead = true;
step_options.ignore_terrain = false;
step_options.nom_step_width = r.nom_step_width;
step_options.nom_forward_step = r.nom_forward_step;
step_options.max_forward_step = r.max_forward_step;
step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;

footsteps = r.createInitialSteps(x0, navgoal, step_options);
for j = 1:length(footsteps)
  footsteps(j).pos = r.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
end
[support_times, supports, comtraj, foottraj, ~, ~] = walkingPlanFromSteps(r, x0, footsteps,step_options);
link_constraints = buildLinkConstraints(r, q0, foottraj);
 
ts = 0:0.1:comtraj.tspan(end);
T = ts(end);

ctrl_data = SharedDataHandle(struct(...
  'comtraj',comtraj,...
  'qtraj',q0,...
  'link_constraints',link_constraints, ...
  'ignore_terrain',false));

qt = QTrajEvalBlock(r,ctrl_data);
aik = ApproximateIKBlock(r,ctrl_data);
% qref = PositionRefFeedthroughBlock(r);
   
% cascade qtraj eval block with approximate IK
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(qt,aik,[],ins,outs);

% % cascade position ref pub
% ins(1).system = 1;
% ins(1).input = 1;
% outs(1).system = 2;
% outs(1).output = 1;
% sys = mimoCascade(sys,qref,[],ins,outs);

xy_offset = [0;0];
toffset = -1;
tt=-1;
while tt<T+1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      xy_offset = x(1:2); % because state estimate will not be 0,0 to start
    end
    tt=t-toffset;

    x = x-xy_offset;
    qdes = sys.output(tt,[],x);
    ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
  end
end

end
