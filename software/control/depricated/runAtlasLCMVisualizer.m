function runAtlasLCMVisualizer

r = RigidBodyManipulator('../../ros_workspace/deprecated/atlas_description/urdf/atlas_robot.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;

runDRCControl(v,'atlas','TRUE_ROBOT_STATE');

% NOTEST