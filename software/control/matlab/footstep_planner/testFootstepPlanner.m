options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
r = r.setInitialState(xstar);
% biped = Biped(r);

% poses = [[0;1;0;0;0;pi/2], [1;1;0;0;0;0], [1;0;0;0;0;0]];
% poses = [[0;1;0;0;0;pi/2], [1;1;0;0;0;0]];
% poses = [1;1;0;0;0;-pi/4];
poses = [1;0;0;0;0;0];
% poses = [0;1;0;0;0;pi/2];
% poses = [1;1;0;0;0;0];
% poses = [0;1;0;0;0;0];
xtraj = r.walkingPlan(xstar, poses, struct('plotting', false, ...
  'interactive', true, 'flat_foot', true));

visualizer = r.constructVisualizer();
visualizer.playback(xtraj, struct('slider', true));