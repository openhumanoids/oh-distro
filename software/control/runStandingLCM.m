function runStandingLCM
%NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

% set initial conditions in gazebo
state_frame = r.getStateFrame();
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);
sys = StandingEndEffectorControl(sys,r,0.2,0.2);

% create support body generator
supp = ConstOrPassthroughSystem(1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot')));
supp = supp.setOutputFrame(sys.getInputFrame.frame{1});

% set up MIMO connections
connection(1).from_output = 1;
connection(1).to_input = 1;
ins(1).system = 2;
ins(1).input = 2;
ins(2).system = 2;
ins(2).input = 3;
ins(3).system = 2;
ins(3).input = 4;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supp,sys,connection,ins,outs);
clear ins;

% create COM goal generator
comg = SimpleCOMGoalGenerator(r);
%comg = SimpleAlternatingCOMGoalGenerator(r);
comg = comg.setOutputFrame(sys.getInputFrame.frame{1});
ins(1).system = 2;
ins(1).input = 2;
ins(2).system = 2;
ins(2).input = 3;
sys = mimoCascade(comg,sys,connection,ins,outs);
clear ins;

% nominal position goal
x0 = r.getInitialState();
qgen = ConstOrPassthroughSystem(x0(7:r.getNumStates()/2));
qgen = qgen.setOutputFrame(sys.getInputFrame.frame{1});
ins(1).system = 2;
ins(1).input = 2;
sys = mimoCascade(qgen,sys,connection,ins,outs);

options.timekeeper = 'drake/lcmTimeKeeper'; 
runLCM(sys,[],options);

end