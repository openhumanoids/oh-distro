function testIKsequence()
options.floating = true;
options.dt = 0.001;

p = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);
load('../../../control/matlab/data/atlas_fp.mat');
p = p.setInitialState(xstar);

ks = ActionSequence();
r_foot = p.findLink('r_foot');
l_foot = p.findLink('l_foot');
r_hand = p.findLink('r_hand');
l_hand = p.findLink('l_hand');

r_foot_contact_pts = r_foot.getContactPoints();
l_foot_contact_pts = l_foot.getContactPoints();
r_hand_contact_pts = mean(r_hand.getContactPoints(),2);

nq = p.getNumDOF();
q0 = xstar(1:nq);
kinsol0 = doKinematics(p,q0);
r_foot_contact_pos = forwardKin(p,kinsol0,r_foot,r_foot_contact_pts,0);
l_foot_contact_pos = forwardKin(p,kinsol0,l_foot,l_foot_contact_pts,0);
r_hand_contact_pos = forwardKin(p,kinsol0,r_hand,r_hand_contact_pts,0);
tspan = [0,1];
kc1 = ActionKinematicConstraint(r_foot,r_foot_contact_pts,r_foot_contact_pos,tspan,'rfoot');
ks = ks.addKinematicConstraint(kc1);
kc2 = ActionKinematicConstraint(l_foot,l_foot_contact_pts,l_foot_contact_pos,tspan,'lfoot');
ks = ks.addKinematicConstraint(kc2);
kc3 = ActionKinematicConstraint(r_hand,r_hand_contact_pts,r_hand_contact_pos+[0.2;0.05;0.7],[tspan(end) tspan(end)],'rhand_ee_goal');
ks = ks.addKinematicConstraint(kc3);
options = struct('nSample',2);
options.q_nom = q0;
options.Q = eye(nq);
[q,t,info] = inverseKinSequence(p,q0,ks,options);
disp(q)
disp(t)
end