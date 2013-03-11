function resetRobotConfig()

options.floating = true;
options.dt = 0.002;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
r = r.setInitialState(xstar);
% set initial conditions in gazebo
state_frame = getStateFrame(r);
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');

end
