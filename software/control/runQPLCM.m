function runQPLCM()

% NOTE: this controller uses the torque control interface, not position
% control.

options.floating = true;
options.dt = 0.001;
r = Atlas('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

% set initial conditions in gazebo
state_frame = r.getStateFrame();
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');

c = QPController(r,xstar(1:r.getNumStates()/2));

options.timekeeper = 'drake/lcmTimeKeeper'; 
runLCM(c,[],options);

end


