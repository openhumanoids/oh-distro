function runAtlasDynamics

% just runs it as a passive system for now

r = RigidBodyManipulator('../../models/atlas_robot.urdf');
v = r.constructVisualizer;
v.display_dt = .05;

sys = cascade(r,v);
sys.setSimulinkParam('MinStep','0.001');

x0 = Point(sys.getStateFrame);
x0.RElbowPitch = -.2;
simulate(sys,[0 10],x0);