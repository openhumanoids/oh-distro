function testHandWorkspace
% test if the HandWorkspace can query the arm posture to achieve closest distance to the
% desired hand pose
options.floating = true;
robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
nomdata = load([getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_fp.mat']);
nq = robot.getNumDOF();
qstar = nomdata.xstar(1:nq);
coords = robot.getStateFrame.coordinates;
coords = coords(1:nq);
l_arm_joints = cellfun(@(s) ~isempty(strfind(s,'l_arm')),coords);
r_arm_joints = cellfun(@(s) ~isempty(strfind(s,'r_arm')),coords);
l_hand = robot.findLinkInd('l_hand');
r_hand = robot.findLinkInd('r_hand');
utorso = robot.findLinkInd('utorso');
hand_space = HandWorkspace([getenv('DRC_PATH'),'/control/matlab/data/HandWorkSpace.mat']);

tic
for i = 1:100
arm_sample_ind = ceil(hand_space.n_samples*rand(6,1));
q_samples = qstar;
q_samples(l_arm_joints) = [hand_space.l_arm_samples(1,arm_sample_ind(1));...
  hand_space.l_arm_samples(2,arm_sample_ind(2));...
  hand_space.l_arm_samples(3,arm_sample_ind(3));...
  hand_space.l_arm_samples(4,arm_sample_ind(4));...
  hand_space.l_arm_samples(5,arm_sample_ind(5));...
  hand_space.l_arm_samples(6,arm_sample_ind(6))];
q_samples(r_arm_joints) = [hand_space.r_arm_samples(1,arm_sample_ind(1));...
  hand_space.r_arm_samples(2,arm_sample_ind(2));...
  hand_space.r_arm_samples(3,arm_sample_ind(3));...
  hand_space.r_arm_samples(4,arm_sample_ind(4));...
  hand_space.r_arm_samples(5,arm_sample_ind(5));...
  hand_space.r_arm_samples(6,arm_sample_ind(6))];
kinsol = robot.doKinematics(q_samples);
utorso_pos = forwardKin(robot,kinsol,utorso,[0;0;0],2);
rhand_pos = forwardKin(robot,kinsol,r_hand,[0;0;0],2);
lhand_pos = forwardKin(robot,kinsol,l_hand,[0;0;0],2);
q_r_arm = hand_space.closestSample(utorso_pos,rhand_pos,false);
q_l_arm = hand_space.closestSample(utorso_pos,lhand_pos,true);
valuecheck(q_r_arm,q_samples(r_arm_joints));
valuecheck(q_l_arm,q_samples(l_arm_joints));
end
toc
end