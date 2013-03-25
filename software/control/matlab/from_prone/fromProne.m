function xtraj = fromProne
% script to test standing up from the prone position

draw_frames=false;

s=warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', struct('floating','true'));
warning(s);
v = r.constructVisualizer();

data = load('../data/aa_atlas_fp.mat');
q = data.xstar(1:getNumDOF(r));

% setup IK prefs
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q;

% find the relevant links:
r_hand = findLink(r,'r_hand');
l_hand = findLink(r,'l_hand');
r_foot = findLink(r,'r_foot');
l_foot = findLink(r,'l_foot');
r_knee = findLink(r,'r_lleg');
l_knee = findLink(r,'l_lleg');
head = findLink(r,'head');

function ps=addbox(p)
  ps = p;
%  ps.min = p-.5*[1;1;0];
%  ps.max = p+.5*[1;1;0];
end

r_hand_pos = [.7;-.25;0;.5;0;pi/2];
l_hand_pos = [.7;.25;0;-.5;0;-pi/2];
r_knee_pos = [-.5; -.18; 0];
l_knee_pos = [-.5; .18; 0];
r_toe_pos.min = [-inf;-.35;0];
r_toe_pos.max = [inf;-0.15;.02];
l_toe_pos.min = [-inf;0.15;0];
l_toe_pos.max = [inf;.35;.02];
com_pos.min = [-inf;-inf;0.1];
com_pos.max = [inf;inf;.2];
head_pos.min = [nan;nan;.35];
head_pos.max = nan(3,1);

ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos};%,r_foot,'toe',r_toe_pos,l_foot,'toe',l_toe_pos};%,0,com_pos};%,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);

ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),repmat(r_toe_pos,1,2),l_foot,'toe',l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape = [0,2];
qtape = [q,q];
if (draw_frames) v.draw(ttape(end),[q;0*q]); end  

com_pos.max = [nan;nan;.25];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,'toe',r_toe_pos,l_foot,'toe',l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 2.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max = [nan;nan;nan];
r_hand_pos = [.4;-.25;.15];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,'toe',r_toe_pos,l_foot,'toe',l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 3;
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_hand_pos = [.2;-.25;0];
kinsol = doKinematics(r,q);
p = forwardKin(r,kinsol,l_foot,mean(getContactPoints(l_foot,'toe'),2));
l_toe_pos.min(1)=p(1);
l_toe_pos.max(1)=p(1);
l_toe_pos.max(3)=0;
l_toe_pos = repmat(l_toe_pos,1,2);
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,'toe',r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 3.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [.4;.25;.15];
com_pos.max(1) = -.2;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,'toe',r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 4;
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [.2;.25;0];
% kinsol = doKinematics(r,q);
r_toe_pos.min(1)=l_toe_pos(1).min(1);
r_toe_pos.max(1)=l_toe_pos(1).min(1);
r_toe_pos.max(3)=0;
r_toe_pos = repmat(r_toe_pos,1,2);
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 4.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_knee_pos=[]; 
r_knee_pos.min = [nan; -.18; 0];
r_knee_pos.max = [nan; -.18; nan];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 5;
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_knee_pos=[];
l_knee_pos.min = [nan; .18; 0];
l_knee_pos.max = [nan; .18; nan];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 5.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1) = -.3;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 6;
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_hand_pos = [-.05;-.25;.15];
com_pos.max(1) = -.4;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 6.5;
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_hand_pos = [-.3;-.25;0];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 7
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [-.05;.2;.15];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 7.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [-.3;.15;0];
com_pos.max(1) = -.45;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 8
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_knee_pos.min = [nan; -inf; 0.05];
r_knee_pos.max = [nan; -.2; inf];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 8.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_knee_pos=[];
l_knee_pos.min = [nan; .2; 0.05];
l_knee_pos.max = [nan; inf; inf];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 9
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_hand_pos = [-.25;-.1;.15];
%r_knee_pos.min = [nan; -.325; 0.1];
%r_knee_pos.max = [nan; -.325; nan];
com_pos.max(1)=-.5;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 9.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

r_hand_pos = [-.3;-.05;0];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 10
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [-.25;.1;.15];
%l_knee_pos.min = [nan; .325; 0.1];
%l_knee_pos.max = [nan; .325; nan];
com_pos.max(1)=-.55;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 10.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

l_hand_pos = [-.3;.05;0];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 11
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

%adding heel constraints
heel_pos.min = [nan;nan;0];
heel_pos.max = [nan;nan;nan];

com_pos.max(1)=-.6;
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 11.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1)=-.7;
r_hand_pos = [-.4;-.1;.1];
l_hand_pos = [-.4;.1;.1];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 12
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1)=-.8;
r_hand_pos = [-.5;-.2;.3];
l_hand_pos = [-.5;.2;.3];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 12.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1)=-.9;
r_hand_pos = [-.7;-.25;.5];
l_hand_pos = [-.7;.25;.5];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 13
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1)=-.9;
heel_pos.min = [nan;nan;0];
heel_pos.max = [nan;nan;0];
r_hand_pos = [-.75;-.25;.55];
l_hand_pos = [-.75;.25;.55];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 13.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

com_pos.max(1)=-.9;
heel_pos.min = [nan;nan;0];
heel_pos.max = [nan;nan;0];
r_hand_pos = [-.8;-.25;.6];
l_hand_pos = [-.8;.25;.6];
ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; %ttape(end) = 14
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

kinsol = doKinematics(r,q);
heel_pos.min = [nan;nan;0];
heel_pos.max = [nan;nan;0];
rpts = forwardKin(r,kinsol,r_foot,getContactPoints(r_foot));
lpts = forwardKin(r,kinsol,l_foot,getContactPoints(l_foot));
pts = [rpts,lpts];  
k = convhull(pts(1:2,:)');
com_pos = mean(pts(:,k),2); com_pos(3)=nan;
% note: also dropping hand and knee constraints
ikargs={r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,0,com_pos,head,zeros(3,1),head_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,q,ikargs{:},options);
ttape(end+1) = ttape(end)+.5; % ttape(end) = 14.5
qtape(:,end+1) = q;
if (draw_frames) v.draw(ttape(end),[q;0*q]); end 

% interpolate one extra frame (because the feet look like they're going
% through the ground)
heel_pos.min = [nan;nan;0];
heel_pos.max = [nan;nan;0];
options.q_nom = mean(qtape(:,(end-1):end),2);
% just foot constraints
ikargs={r_foot,getContactPoints(r_foot,'toe'),r_toe_pos,l_foot,getContactPoints(l_foot,'toe'),l_toe_pos,r_foot,'heel',heel_pos,l_foot,'heel',heel_pos};
q=inverseKin(r,options.q_nom,ikargs{:},options);
ttape(end+1)=ttape(end);
ttape(end-1) = mean(ttape((end-2):(end-1)),2);
qtape(:,end+1)=qtape(:,end);
qtape(:,end-1)=q;

xtraj = PPTrajectory(foh(ttape,[qtape;0*qtape]));
xtraj = setOutputFrame(xtraj,r.getStateFrame);
v.playback(xtraj,struct('slider',true));

%v.playbackAVI(xtraj,'fromProne.avi');

end
