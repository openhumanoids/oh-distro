function runAtlasDynamics

% just runs it as a passive system for now

r = RigidBodyManipulator('../../ros_workspace/atlas_description/urdf/atlas_robot.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RElbowPitch = -.2;

if (1)
  % Run animation while it is simulating (as fast as possible, but probably
  % slower than realtime)
  sys = cascade(r,v);
  simulate(sys,[0 10],x0); 
else
  % Run simulation, then play it back at realtime speed
  xtraj = simulate(r,[0 5],x0);
  v.playback(xtraj);
end