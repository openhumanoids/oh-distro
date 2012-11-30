function runAtlasPlanarDynamics()

% just runs it as a passive system for now
options.view = 'right';
options.view = 'front';

options.floating = true;
m = PlanarRigidBodyModel('../models/mit_gazebo_models/mit_robot_drake/mit_drc_robot_minimal_contact.sdf',options);

dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);
r = setSimulinkParam(r,'MinStep','0.0001');
v = r.constructVisualizer;
v.display_dt = .01;

x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r.manip,double(x0));

if (1)
  % Run animation while it is simulating (as fast as possible, but probably
  % slower than realtime)
  s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  sys = cascade(r,v);
  warning(s);
  simulate(sys,[0 10],x0); 
else
  % Run simulation, then play it back at realtime speed
  xtraj = simulate(r,[0 5],x0);
  v.playback(xtraj);
end
