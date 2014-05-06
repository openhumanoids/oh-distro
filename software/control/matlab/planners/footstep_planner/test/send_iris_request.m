options.floating = true;
options.dt = 0.001;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

request = drc.iris_region_request_t();
request.utime = 0;

fp = load(strcat(getenv('DRC_PATH'), '/control/matlab/data/atlas_fp.mat'));
fp.xstar(3) = fp.xstar(3) + 0.50; % make sure we're not assuming z = 0
request.initial_state = r.getStateFrame().lcmcoder.encode(0, fp.xstar);

request.num_seed_poses = 3;
request.seed_poses = javaArray('drc.position_3d_t', request.num_seed_poses);
poses = [0.1 -0.2 0.1;
         -.6 -.6 -1;
         0 -pi/4 pi/4];
for j = 1:request.num_seed_poses
  request.seed_poses(j) = encodePosition3d([poses(1:2,j); 0;0;0;poses(3,j)]);
end
lc = lcm.lcm.LCM.getSingleton();
lc.publish('IRIS_REGION_REQUEST', request);
  