function runAtlasPDcontrol
r = RigidBodyManipulator('../../ros_workspace/atlas_description/urdf/atlas_robot.urdf');
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
xd = x0;
xd.RElbowPitch = -0.2;
xd.RShoulderPitch = 0.1;
%%%%
num_x = r.getNumContStates();
num_u = r.getNumInputs();
xdDouble = zeros(num_x,1);
xdDouble(15)= -0.2;
xdDouble(13)=0.1;
PDcontrol = simplePDcontrol(num_x,num_u,xdDouble);
PDcontrol = PDcontrol.setInputFrame(r.getOutputFrame());
PDcontrol = PDcontrol.setOutputFrame(r.getInputFrame());
options = struct('tspan',[0,5]);
runDRCControl(PDcontrol,'atlas','TRUE_ROBOT_STATE','ACTUATOR_CMDS',options);
keyboard;
end

% NOTEST