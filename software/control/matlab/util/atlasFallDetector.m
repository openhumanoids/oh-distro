function atlasFallDetector(publish_lcmgl)
%NOTEST

if nargin < 1
  publish_lcmgl = false;
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
options.floating = true;
options.ignore_friction = true;
options.atlas_version = 5;

r = DRCAtlas([],options);
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);

fallDetector = FallDetectorTriangle(r, publish_lcmgl);
fallDetector.run();
disp('done?');

end
