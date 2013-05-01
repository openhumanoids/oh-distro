function drakeBalancing

% put robot in a random x,y,yaw position and balance for 5 seconds
visualize = true;

addpath(fullfile(pwd,'..'));
addpath(fullfile(pwd,'../frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

nq = getNumDOF(r);
nu = getNumInputs(r);

% set initial state to fixed point
load('../data/atlas_fp.mat');
xstar(1) = 10*randn();
xstar(2) = 10*randn();
xstar(6) = pi*randn();
xstar(nq+1) = 0.1;
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% build TI-ZMP controller 
foot_pos = contactPositions(r,q0); 
ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
comgoal = mean(foot_pos(1:2,ch),2);
limp = LinearInvertedPendulum(com(3));
[~,V] = lqr(limp,comgoal);

foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));

ctrl_data = SharedDataHandle(struct('A',[zeros(2),eye(2); zeros(2,4)],...
      'B',[zeros(2); eye(2)],'C',[eye(2),zeros(2)],'D',[],...
      'R',zeros(2),'Qy',eye(2),'S',V.S,'s1',zeros(4,1),'xlimp0',[comgoal;0;0],...
      'qtraj',q0,'supptraj',foot_support));           
           
           
% instantiate QP controller
options.slack_limit = 30.0;
options.w = 0.1;
options.R = 1e-12*eye(nu);
act_idx = getActuatedJoints(r);
joint_names = getJointNames(r);
joint_names = joint_names(2:end); % get rid of null string at beginning..
ankle_idx = ~cellfun(@isempty,strfind(joint_names,'lax')) | ~cellfun(@isempty,strfind(joint_names,'uay'));
ankle_idx = find(ankle_idx(act_idx));
options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx); % soft ankles
options.lcm_foot_contacts = false;
qp = QPController(r,ctrl_data,options);
clear options;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% feedback PD trajectory controller 
options.q_nom = q0;
pd = SimplePDController(r,ctrl_data,options);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],[],outs);
clear outs;

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.05;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 5],x0);
if visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end


end
