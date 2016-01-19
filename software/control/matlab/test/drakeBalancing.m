function drakeBalancing(use_angular_momentum)

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','ZMP'));

% put robot in a random x,y,yaw position and balance for 3 seconds
visualize = true;

if (nargin<1); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
options.ignore_friction = 1;
options.atlas_version = 5;
r = DRCAtlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load(r.fixed_point_file);
xstar(1) = 10*randn();
xstar(2) = 10*randn();
xstar(6) = pi*randn();
%xstar(nq+1) = 0.1;
r = r.setInitialState(xstar);

x0 = xstar;
% Construct plan
settings = QPLocomotionPlanSettings.fromStandingState(x0, r);

% Only use supports when in contact
settings.planned_support_command = QPControllerPlan.support_logic_maps.kinematic_or_sensed;
standing_plan = QPLocomotionPlanCPPWrapper(settings);

param_sets = atlasParams.getDefaults(r);
if use_angular_momentum
  param_sets.standing = StandingAngularMomentum(r);
end

planeval = bipedControllers.BipedPlanEval(r, standing_plan);
control = bipedControllers.InstantaneousQPController(r, param_sets);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);

sys = feedback(r, plancontroller);

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

traj = simulate(sys,[0 3],x0);
if visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end


end
