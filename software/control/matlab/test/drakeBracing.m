function drakeBracing()

% put robot in a random x,y,yaw position and balance for 3 seconds
visualize = true;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.001;
options.ignore_friction = 0;
options.atlas_version = 4;
urdf = strcat(getenv('DRC_PATH'),'/models/atlas_v4/model_convex_hull_closed_hands.urdf');
r = DRCAtlas(urdf,options);

nq = getNumPositions(r);
nv = getNumVelocities(r);

% set initial state to fixed point
load(r.fixed_point_file);
xstar(1) = 1*randn();
xstar(2) = 1*randn();
xstar(3) = xstar(3)+0.01;
xstar(6) = pi*randn();
xstar(nq+1) = 0.3*randn(); % randomly falling in some direction
xstar(nq+2) = 0.3*randn();
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);

% construct trajectory from current to desired bracing configuration
d = load(fullfile(getenv('DRC_PATH'),'/control/matlab/data/atlas_bracing_v5.mat'));
tf = 0.8;
qf = d.xstar(1:nq);
qtraj = PPTrajectory(foh([0 tf],[q0,qf]));

ctrl_data = AtlasPositionControllerData(struct('qtraj',qtraj));

options.use_actuator_coordinates = true;
qt = QTrajEvalBlock(r,ctrl_data,options);

nu = getNumInputs(r);
Kp = 150;
Kd = getDampingGain(Kp,0.3);
pd_atlas = pdcontrol(r,Kp*eye(nu),Kd*eye(nu),r.getActuatedJoints);
pd_atlas = pd_atlas.setInputFrame(atlasFrames.AtlasInput(r));
sys = mimoFeedback(qt,pd_atlas);

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.05;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end

traj = simulate(sys,[0 2.5],x0);
if visualize
  playback(v,traj,struct('slider',true));
end

ts = traj.getBreaks();
[jlmin,jlmax] = getJointLimits(r);

joint_limit_hit = zeros(nq,1);
max_abs_vel = zeros(nv,1);

for i=1:length(ts)
  x = traj.eval(ts(i));
  q = x(1:nq);
  qd = x(nq+(1:nv));
  joint_limit_hit = joint_limit_hit | (q<=jlmin+1e-4) | (q>=jlmax-1e-4); 
  max_abs_vel = max(abs(qd),max_abs_vel);
end

coords =r.getStateFrame.coordinates;
jlstrings = coords(joint_limit_hit);

fprintf('\nJoint limits hit on : ');
for i=1:length(jlstrings)
  fprintf([jlstrings{i} ', ']);
end
fprintf('\n\n');

x_knots = traj.eval(ts);
N = 5;
fprintf('The %d highest absolute velocities were...\n',N);
[max_abs_vel,ind] = sort(max_abs_vel,1,'descend');
for i=1:N
  fprintf([coords{ind(i)} ' : ' num2str(max_abs_vel(i)) '\n']);
  figure(123+i);
  title(coords{ind(i)});
  subplot(2,1,1)
  plot(ts,x_knots(ind(i),:),'b.-');
  xlabel('rad');
  xlabel('time (s)');
  title(regexprep(coords{ind(i)},'_',' '));
  subplot(2,1,2)
  plot(ts,x_knots(ind(i)+nq,:),'r.-');
  xlabel('rad/s');
  xlabel('time (s)');
  title([regexprep(coords{ind(i)},'_',' ') 'velocity']);
end



end
