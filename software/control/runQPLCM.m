function runQPLCM()

% NOTE: this controller uses the torque control interface, not position
% control.

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

v = r.constructVisualizer;
v.display_dt = 0.01;

nx = r.getNumStates();
nq = nx/2;

load('data/atlas_fp.mat');

c = QPController(r,xstar(1:nq));

options.timekeeper = 'drake/lcmTimeKeeper'; 
runLCM(c,[],options);

end


