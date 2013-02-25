function fromProne
% script to test standing up from the prone position

r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_ros.urdf', struct( ...
  'floating','true')); %, ...
%  'package','/Users/russt/drc/ros_workspace/mit_drcsim_scripts/models/mit_robot'));
v = r.constructVisualizer();

data = load('data/aa_atlas_fp.mat');
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

r_hand_pos = [.3;-.2;0];
l_hand_pos = [.3;.2;0];
r_knee_pos = [-.5; -.1; 0];
l_knee_pos = [-.5; .1; 0];
r_toe_pos.min = [-inf;-inf;0];
r_toe_pos.max = [inf;0;inf];
l_toe_pos.min = [-inf;0;0];
l_toe_pos.max = [inf;inf;inf];
com_pos.min = [-inf;-inf;0.1];
com_pos.max = [inf;inf;inf];
head_pos.min = [nan;nan;.2];
head_pos.max = nan(3,1);

ikargs={r_hand,'default',r_hand_pos,l_hand,'default',l_hand_pos,r_knee,'default',r_knee_pos,l_knee,'default',l_knee_pos,r_foot,'toe',r_toe_pos,l_foot,'toe',l_toe_pos,0,com_pos};%,head,zeros(3,1),head_pos};
q=inverseKin(r,q,ikargs{:},options);
v.draw(0,[q;0*q]);

end
