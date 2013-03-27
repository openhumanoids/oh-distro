options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
r = r.setInitialState(xstar);
nq = getNumDOF(r);
x0 = xstar;
qstar = xstar(1:nq);
% biped = Biped(r);

% poses = [[0;1;0;0;0;pi/2], [1;1;0;0;0;0], [1;0;0;0;0;0]];
% poses = [[0;1;0;0;0;pi/2], [1;1;0;0;0;0]];
% poses = [3;3;0;0;0;-pi/4];
poses = [1;0;0;0;0;0];
% poses = [0; -.4; 0;0;0;0];
% poses = [0;1;0;0;0;pi/2];
% poses = [1;1;0;0;0;0];
% poses = [0;1;0;0;0;0];
xtraj = r.walkingPlan(x0, qstar, [poses(:,end); 30]);

visualizer = r.constructVisualizer();
visualizer.playback(xtraj, struct('slider', true));