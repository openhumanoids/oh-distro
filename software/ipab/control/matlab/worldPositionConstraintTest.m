% This test if the world link is added to the collision pairs during the
% collision check. The variable "valid" should be false.

options = struct();
options.floating = true;

urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_chull.urdf');
robot = RigidBodyManipulator(urdf,options);
world_link = robot.findLinkId('world');
l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
l_larm = robot.findLinkId('l_larm');

robot = Scenes.generate(statVars.options, robot, world_link);
robot = robot.compile();

tree = simVars.T_smooth;

q = extractPath(tree, simVars.path_ids_A);
q = q(tree.idx{tree.cspace_idx},:);

v = robot.constructVisualizer();
s = '';
i = 1;
while ~strcmp(s, 'q')
    v.draw(0, q(1:36,i));
    s = input('','s');
    if strcmp(s,'')
        i = i+1;
    else
        i = i-1;
    end
    if i > size(q,2)
        i = size(q,2);
    elseif i < 1
        i = 1;
    end
end 

% position = [0.;-0.4;0.7];
% constraint = WorldPositionConstraint(robot, robot.findLinkId('r_hand'), [0; 0; 0],  position ,  position, [0.0, 1.0]); 
% % constraint = Point2PointDistanceConstraint(robot, 'world', 'r_hand', position, [0;0;0], 0, 0);
% 
% %Set IK options
% cost = Point(robot.getPositionFrame(),10);
% for i = robot.getNumBodies():-1:1
%   if all(robot.getBody(i).parent > 0) && all(robot.getBody(robot.getBody(i).parent).position_num > 0)
%     cost(robot.getBody(robot.getBody(i).parent).position_num) = ...
%       cost(robot.getBody(robot.getBody(i).parent).position_num) + cost(robot.getBody(i).position_num);
%   end
% end
% cost = cost/min(cost);
% Q = diag(cost);
% ikoptions = IKoptions(robot);
% ikoptions = ikoptions.setMajorIterationsLimit(100);
% ikoptions = ikoptions.setQ(Q);
% ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
% 
% [q, info, infeasible_constraint] = inverseKin(robot, q, q, constraint, ikoptions);
% v.draw(0, q);
% drawLinkFrame(robot, 'r_hand', q, 'Right Hand Frame');
% drawFrame([eye(3) position; [0 0 0 1]], 'Right hand target');
% kinsol = robot.doKinematics(q);
% footPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
% rotmat = quat2rotmat(footPose(4:7));