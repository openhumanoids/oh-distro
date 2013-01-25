function [r,v,sys] = runGraspTest()

dt=0.001;
options.floating=false; % If this is set to true there are issues, need to fix them later
path=getenv('DRC');
filelocation=[path 'mit_robot_hands/drake_urdfs/sis_sandia_hand_right_drake.sdf'];
r=TimeSteppingRigidBodyManipulator(filelocation,dt,options);
v = r.constructVisualizer();
v.display_dt = 0.01;

% actuator_names = r.getInputFrame.coordinates;
% upublisher = ActuatorCmdPublisher('sandia_hand',actuator_names,'ACTUATOR_CMDS');;
% xframe = SandiaState(r);

[kp,kd] = getHandPDGains(r); 
% [pdfw,pdfb] = pdcontrol(r,kp,kd);


% joint_names=r.getStateFrame.coordinates(1:r.getNumStates()/2);
% % Setting up LCM
% hand_state_coder = RobotStateCoder('sandia_hand',joint_names);
% hand_state=LCMCoordinateFrameWCoder('SandiaState',r.getNumStates(),'x',JLCMCoder(hand_state_coder));
% hand_state.setCoordinateNames(r.getStateFrame.coordinates);
% hand_state.setDefaultChannel('EST_HAND_STATE');
% 
% r.setStateFrame(hand_state);
% r.setOutputFrame(hand_state);

% [kp,kd] = getHandPDGains(r); 
% sig=SimpleGraspBoolGenerator();

% sig=trueFalseToOneZero();
% traj=ConstantTrajectory(true);
% traj=setOutputFrame(traj,sig.getInputFrame);
% c1=cascade(traj,sig);
% c2=ClosedGraspController(r);
% c2=c2.setInputFrame(c1.getOutputFrame);
% pd = pdcontrol(r,kp,kd);

% Bypassing things to simplify system
% sig=trueFalseToOneZero();
% traj=ConstantTrajectory(1);
% c1=cascade(traj,sig);
c=ClosedGraspController(r);
% traj=setOutputFrame(traj,c2.getInputFrame);
% c2=c2.setInputFrame(c1.getOutputFrame);
pd = pdcontrol(r,kp,kd);

% sys=cascade(traj,c2);
% pdfw.setInputFrame(sys.getOutputFrame);
pd=pd.setInputFrame(c.getOutputFrame);
sys=cascade(cascade(c,pd),v);
% pd=setInputFrame(pd,sys1.getOutputFrame);
% sys2=cascade(sys1,pd);

runLCM(sys)
% sys.runLCM()

% x0 = Point(sys2.getStateFrame);
% x0.base_z=.5;
% x0 = r.manip.resolveConstraints(double(x0));

% sys = pd;


% T = 1.0; % sec
% if (0)
%     tic;
%     traj = simulate(sys2,[0 T]); 
%     toc;
%     playback(v,traj,struct('slider',true));
% else
%     s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
%     sys = cascade(sys,v);
%     warning(s);
%     simulate(sys,[0 T]);
% end

end