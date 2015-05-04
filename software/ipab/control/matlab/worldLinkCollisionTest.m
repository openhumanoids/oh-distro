% This test if the world link is added to the collision pairs during the
% collision check. The variable "valid" should be false.

options = struct();
options.floating = true;

urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_convex_hull_fingers.urdf');
robot = RigidBodyManipulator(urdf,options);
q = [-0.0063; -0.1556; 0.8347; 0.0249; 0.0152; -0.0922; -0.4990; -0.0082;...
          -0.2088; 0.1868; -0.1484; 1.9012 ;2.3547; 0.4514 ;0.0943; 0.2290;...
          -0.4904 ;0.9780 ;-0.5005 ;-0.2596 ;-1.0792; -0.2700 ;1.3300 ;2.1000;...
          -0.5000; 0; 0.0922; 0.1209; -0.6036; 1.1557; -0.5650 ;-0.1404; 0; 0];
world_link = robot.findLinkId('world');
l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
l_larm = robot.findLinkId('l_larm');
obstacle = RigidBodyBox([.3 .3 .4], [.55 .35 1.1125], [0 0 0]);
robot = addGeometryToBody(robot, world_link, obstacle);
robot = robot.compile();
active_collision_options.body_idx = [world_link, l_larm];

v = robot.constructVisualizer();
v.draw(0, q);

phi = robot.collisionDetect(q, false, active_collision_options);
valid = all(phi > 0);
