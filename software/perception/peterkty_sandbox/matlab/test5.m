pt_world = [0,0,0]';


state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');
[x,t] = state_frame.getNextMessage(10);

rhand = r.findLinkInd('r_hand');

