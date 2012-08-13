function runAtlasLCMVisualizer

r = RigidBodyManipulator('../../ros_workspace/atlas_description/urdf/atlas_robot.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;

runDRCLCMControl(v,'atlas','true_robot_state');
