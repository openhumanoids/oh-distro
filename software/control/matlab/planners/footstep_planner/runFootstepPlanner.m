function runFootstepPlanner(options)
% NOTEST

while 1
  if (nargin < 1); options = struct(); end
  if ~isfield(options, 'enable_terrainmaps'); options.enable_terrainmaps = true; end
  options.floating = true;
  options.dt = 0.001;
  
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
  options.visual = false; % loads faster
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
  r = removeCollisionGroupsExcept(r,{'heel','toe'});
  if options.enable_terrainmaps
    r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'fill', true,'normal_radius',2,'normal_method','ransac','auto_request',true)));
  end
  r = compile(r);

  p = FootstepPlanner(r);
  try
    p.run(0.25)
  catch exception
    disp(exception.getReport())
    save(['footstep-planner-crash-',datestr(now(),'YYYY-mm-DDTHH-MM'),'.mat'], 'exception');
    msg ='Foot Plan : Crashed, restarting...'; disp(msg); send_status(6,0,0,msg);
    clear all classes java imports mex
  end
end
