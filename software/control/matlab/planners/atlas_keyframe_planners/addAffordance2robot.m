function robot = addAffordance2robot(robot,urdf,xyz,rpy,options)
% add urdf to robot, and set clear the collision checking between affordance. only leave
% the collision checking between robot to affordance, and robot self collision
% same input list as addRobotFromURDF except options
robot = robot.addRobotFromURDF(urdf,xyz,rpy,options);
% group each body into robot/affordance
num_bodies = robot.getNumBodies;
body2robotnumMap = zeros(num_bodies,1);
for i = 1:num_bodies
  body2robotnumMap(i) = robot.getBody(i).robotnum;
end
body_ind = (1:num_bodies)';
aff_collision_group = {};
for i = 2:length(robot.name)
  if(~robot.collision_filter_groups.isKey(robot.name{i}))
    robot.collision_filter_groups(robot.name{i}) = CollisionFilterGroup();
    robot = robot.addLinksToCollisionFilterGroup(body_ind(body2robotnumMap == i),robot.name{i},i);
  end
  robot = compile(robot);
  aff_collision_group = [aff_collision_group,robot.name(i)];
end
for i = 2:length(robot.name)
  robot = robot.addToIgnoredListOfCollisionFilterGroup(aff_collision_group,robot.name{i});
  robot = compile(robot);
end
end