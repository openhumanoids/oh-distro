function runAtlasSagittalDynamics

% just runs it as a passive system for now

options.view = 'right';
options.floating = true;
m = PlanarRigidBodyModel('../../ros_workspace/deprecated/atlas_description/urdf/atlas_robot.urdf',options);
r = TimeSteppingRigidBodyManipulator(m,.01);
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
x0.RElbowPitch = -.2;
x0 = resolveConstraints(r.manip,double(x0));

if (0)
  % Run animation while it is simulating (as fast as possible, but probably
  % slower than realtime)
  s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  sys = cascade(r,v);
  warning(s);
  simulate(sys,[0 10],x0); 
else
  % Run simulation, then play it back at realtime speed
  xtraj = simulate(r,[0 3],x0);
  save saggital.mat v xtraj;
  v.playback(xtraj);
end
