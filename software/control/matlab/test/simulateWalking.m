function traj = simulateWalking(r, walking_ctrl_data, ts, use_mex, use_ik, use_bullet, use_angular_momentum, draw_button)
%NOTEST

addpath(fullfile(getDrakePath,'examples','ZMP'));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')


% BotVisualizer doesn't support DRC terrains yet
terrain = r.getTerrain();
r = r.setTerrain([]);
r = r.compile();
v = r.constructVisualizer;
r = r.setTerrain(terrain);
r = r.compile();
v.display_dt = 0.05;

x0 = r.getInitialState();
nq = getNumPositions(r);

T = ts(end);

ctrl_data = QPControllerData(true,struct(...
  'acceleration_input_frame',AtlasCoordinates(r),...
  'D',-getAtlasNominalCOMHeight()/9.81*eye(2),... % assumed COM height
  'Qy',eye(2),...
  'S',walking_ctrl_data.S,... % always a constant
  's1',walking_ctrl_data.s1,...
  's2',walking_ctrl_data.s2,...
  's1dot',walking_ctrl_data.s1dot,...
  's2dot',walking_ctrl_data.s2dot,...
  'x0',ConstantTrajectory([walking_ctrl_data.zmptraj.eval(T);0;0]),...
  'u0',ConstantTrajectory(zeros(2,1)),...
  'qtraj',x0(1:nq),...
  'comtraj',walking_ctrl_data.comtraj,...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',walking_ctrl_data.supports,...
  'mu',walking_ctrl_data.mu,...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'y0',walking_ctrl_data.zmptraj,...
  'plan_shift',zeros(3,1),...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));

options.dt = 0.003;
options.use_bullet = use_bullet;
options.debug = false;
options.use_mex = use_mex;

if use_angular_momentum
  options.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  options.W_kdot = 1e-5*eye(3); % angular momentum weight
end

haltBlock = HaltSimulationBlock(r.getOutputFrame(), draw_button);
haltBlock = haltBlock.setInputFrame(r.getOutputFrame());
haltBlock = haltBlock.setOutputFrame(r.getOutputFrame());

if (use_ik)
  options.w_qdd = 0.001*ones(nq,1);
  % instantiate QP controller
  qp = QPController(r,{},ctrl_data,options);

  % feedback QP controller with atlas
  ins(1).system = 1;
  ins(1).input = 2;
  ins(2).system = 1;
  ins(2).input = 3;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(qp,cascade(r, haltBlock),[],[],ins,outs);
  clear ins;

  % feedback foot contact detector with QP/atlas
  options.use_lcm=false;
  fc = FootContactBlock(r,ctrl_data,options);
  ins(1).system = 2;
  ins(1).input = 1;
  sys = mimoFeedback(fc,sys,[],[],ins,outs);
  clear ins;  
  
  % feedback PD block
  pd = IKPDBlock(r,ctrl_data,options);
  ins(1).system = 1;
  ins(1).input = 1;
  sys = mimoFeedback(pd,sys,[],[],ins,outs);
  clear ins;

else
  
  options.Kp_foot = [100; 100; 100; 150; 150; 150];
  options.foot_damping_ratio = 0.5;
  options.Kp_pelvis = [0; 0; 150; 200; 200; 200];
  options.pelvis_damping_ratio = 0.6;
  options.Kp_q = 150.0*ones(r.getNumPositions(),1);
  options.q_damping_ratio = 0.6;

  % construct QP controller and related control blocks
  [qp,lfoot_controller,rfoot_controller,pelvis_controller,pd,options] = constructQPWalkingController(r,ctrl_data);

  % feedback QP controller with atlas
  ins(1).system = 1;
  ins(1).input = 2;
  ins(2).system = 1;
  ins(2).input = 3;
  ins(3).system = 1;
  ins(3).input = 4;
  ins(4).system = 1;
  ins(4).input = 5;
  ins(5).system = 1;
  ins(5).input = 6;
  outs(1).system = 2;
  outs(1).output = 1;
  % sys = mimoFeedback(qp,r,[],[],ins,outs);
  sys = mimoFeedback(qp,cascade(r, haltBlock), [], [], ins, outs);
  clear ins outs;
  
  % feedback foot contact detector with QP/atlas
  options.use_lcm=false;
  fc = FootContactBlock(r,ctrl_data,options);
  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  ins(3).system = 2;
  ins(3).input = 4;
  ins(4).system = 2;
  ins(4).input = 5;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(fc,sys,[],[],ins,outs);
  clear ins outs;  
  
  % feedback PD block
  ins(1).system = 1;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 2;
  ins(3).system = 2;
  ins(3).input = 3;
  ins(4).system = 2;
  ins(4).input = 4;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(pd,sys,[],[],ins,outs);
  clear ins outs;

  % feedback body motion control blocks
  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  ins(3).system = 2;
  ins(3).input = 4;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(lfoot_controller,sys,[],[],ins,outs);
  clear ins outs;

  ins(1).system = 2;
  ins(1).input = 1;
  ins(2).system = 2;
  ins(2).input = 3;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(rfoot_controller,sys,[],[],ins,outs);
  clear ins outs;

  ins(1).system = 2;
  ins(1).input = 1;
  outs(1).system = 2;
  outs(1).output = 1;
  sys = mimoFeedback(pelvis_controller,sys,[],[],ins,outs);
  clear ins outs;
end

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);

end
