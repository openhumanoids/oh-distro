function runReachingLCM
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

% create end effectors
joint_names = r.getJointNames();
joint_names = joint_names(2:end); % get rid of null string at beginning..
right_ee = EndEffector(r,'atlas','r_hand',[0;0;0],'R_HAND_GOAL');
right_ee = right_ee.setMask(~cellfun(@isempty,strfind(joint_names,'r_arm')));
left_ee = EndEffector(r,'atlas','l_hand',[0;0;0],'L_HAND_GOAL');
left_ee = left_ee.setMask(~cellfun(@isempty,strfind(joint_names,'l_arm')));

% add end effectors
sys = sys.addEndEffector(right_ee);
sys = sys.addEndEffector(left_ee);

% create support body generator
supp = ConstOrPassthroughSystem(1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot')));
supp = supp.setOutputFrame(sys.getInputFrame.frame{3});
connection(1).from_output = 1;
connection(1).to_input = 3;
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
ins(5).system = 2;
ins(5).input = 6;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supp,sys,connection,ins,outs);
clear connection ins;

% COM goal generator
comg = SimpleCOMGoalGenerator(r);
comg = comg.setOutputFrame(sys.getInputFrame.frame{3});
connection(1).from_output = 1;
connection(1).to_input = 3;
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 4;
ins(4).system = 2;
ins(4).input = 5;
sys = mimoCascade(comg,sys,connection,ins,outs);
clear connection ins;

% nominal position goal
x0 = r.getInitialState();
qgen = ConstOrPassthroughSystem(x0(7:r.getNumStates()/2));
qgen = qgen.setOutputFrame(sys.getInputFrame.frame{3});
connection(1).from_output = 1;
connection(1).to_input = 3;
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 4;
sys = mimoCascade(qgen,sys,connection,ins,outs);

options.timekeeper = 'drake/lcmTimeKeeper'; 
runLCM(sys,[],options);

end