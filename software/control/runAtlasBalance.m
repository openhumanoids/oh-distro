function runAtlasBalance()

options.floating = true;
m = RigidBodyModel('../models/mit_gazebo_models/mit_robot_drake/mit_drc_robot_minimal_contact.sdf',options);
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);
r = setSimulinkParam(r,'MinStep','0.001');

nq = r.manip.getNumStates()/2;
x0 = Point(r.getStateFrame);
xstar = r.manip.resolveConstraints(double(x0));

c = COMController(r,xstar(1:nq));
runDRCControl(r,c,'mit_drc_robot','TRUE_ROBOT_STATE','ACTUATOR_CMDS',struct());

end

