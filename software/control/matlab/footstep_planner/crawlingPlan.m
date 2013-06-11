function [qtraj,support_times,supports,S,s1,s2,comtraj,zmptraj] = crawlingPlan(r,x0,foot_spec,options)
%
% @param r the robot 
% @param x0 initial state
% @param footspec 4 element struct with footspec(i).body_ind
%                                       footspec(i).contact_pt  % 3x1
%
% @option num_steps will be rounded up to be a multiple of 4
% @option step_speed in m/s
% @option step_height in m
% @option ignore_terrain
% @options direction - 0 for forward, <0 for left, >0 for right 
% @options gait - 0 for walk, 1 for trot

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
sizecheck(x0,[getNumStates(r) 1]);
nq = getNumDOF(r);
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

sizecheck(foot_spec,4);
fieldcheck(foot_spec,'body_ind');
fieldcheck(foot_spec,'contact_pt');
for i=1:4, sizecheck(foot_spec(i).contact_pt,[3 1]); end

if nargin<3 || isempty(options), options=struct(); end
if ~isfield(options,'num_steps') options.num_steps = 8; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'qnom') options.qnom = q0; end
if ~isfield(options,'gait') options.gait = 0; end

% always take 4 steps at a time
options.num_steps = 4*ceil(options.num_steps/4);

[zmptraj,foottraj, support_times, supports] = planInitialZMPTraj(biped, q0, footsteps, footstep_opts);
zmptraj = setOutputFrame(zmptraj,desiredZMP);
%% construct ZMP feedback controller
com = getCOM(biped,kinsol);
foot_pos = contactPositions(biped,kinsol,biped.foot_bodies_idx);
zfeet = mean(foot_pos(3,:));

% get COM traj from desired ZMP traj
options.com0 = com(1:2);
[c,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);
