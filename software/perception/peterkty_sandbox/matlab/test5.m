pt_world = [0,0,0]';

r= Atlas;
state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');
[x,t] = state_frame.getNextMessage(10);

rhand = r.findLinkInd('r_hand');

pt_world = [0,0,0]';
pt_hand = r.bodyKin(kinsol, rhand, pt_world);
kinsol = r.doKinematics(x(1:r.getNumDOF()));
pt_hand = r.bodyKin(kinsol, rhand, pt_world);


v = r.constructVisualizer;
v.draw(0, r.x0)



RigidBodyFrame
edit RigidBodyFrame.m

r = r.addFrame(RigidBodyFrame(rhand,[1;0;0],[0;0;0],'palm'))
palm = r.findFrameId('palm')
pt_hand = r.bodyKin(kinsol, palm, pt_world);
r = compile(r);
kinsol = r.doKinematics(r.x0(1:r.getNumDOF()))
pt_hand = r.bodyKin(kinsol, palm, pt_world);