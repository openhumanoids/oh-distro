classdef Scenes
  
  methods(Static)
    
    function names = getSceneNames(scenes)
      names = {'scene1', 'scene2', 'scene3'};
      if nargin == 1
        names = names{scenes};
      end
    end
    
    function nScenes = getnScenes()
      nScenes = length(Scenes.getSceneNames());
    end
    
    function robot = generateRobot(options)
      switch options.model
        case 'v3'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_chull.urdf');
          else
            urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_full.urdf');
          end
        case 'v4'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_chull.urdf');
          else
            urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_full.urdf');
          end
        case 'v5'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_chull.urdf');
          else
            urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_full.urdf');
          end
        case 'val'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_simplified.urdf');
          else
%             urdf = fullfile(getDrakePath(),'..','models','valkyrie','oldValkyrie.urdf');
            urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
          end
          %     case 'lwr'
          %         urdf = fullfile(getDrakePath(),'..','models','lwr_defs','robots','lwr_drake.urdf');
        otherwise
          error('Unknown robot model "%s"\n', options.model);
      end
      robot = RigidBodyManipulator(urdf,options);      
    end
    
    function robot = generateWorld(options, robot, world_link)
      switch options.scene
        case 0 %debris
          collision_object = RigidBodyCapsule(0.05,1,[0.95,0.22,0.35],[0,pi/2,0]);
          collision_object.c = [0.5;0.4;0.3];
          robot = addGeometryToBody(robot, world_link, collision_object);
          
          collision_object = RigidBodyCapsule(0.05,1,[0.95,-0.22,0.35],[0,pi/2,0]);
          collision_object.c = [0.5;0.4;0.3];
          robot = addGeometryToBody(robot, world_link, collision_object);
          
          collision_object = RigidBodyCapsule(0.05,1,[0.8,-0.05,0.35],[-pi/4,0,0]);
          collision_object.c = [0.5;0.4;0.3];
          robot = addGeometryToBody(robot, world_link, collision_object);
          
          collision_object = RigidBodyCapsule(0.05,1,[0.45,-0.05,0.35],[-pi/4,0,0]);
          collision_object.c = [0.5;0.4;0.3];
          robot = addGeometryToBody(robot, world_link, collision_object);
          
          l_hand = robot.findLinkId('l_hand');
          collision_object = RigidBodyCapsule(0.05,1, [0.35,0.27,0],[0,pi/2,0]);
          collision_object.c = [0.5;0.4;0.3];
          robot = addGeometryToBody(robot, l_hand, collision_object);
          
          collision_object = RigidBodyCapsule(0.05,1,[0;0;0],[0,pi/2,0]);
          collision_object.c = [0.5;0.4;0.3];
        case 1
          table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);          
        case 2
          table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          obstacle = RigidBodyBox([.3 .3 .4], [.55 .35 1.1125], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle);
          
          obstacle = RigidBodyBox([.3 .3 .4], [.55 -0.35 1.1125], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle);
          
          obstacle = RigidBodyBox([.3 1 .4], [.55 0 1.5125], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);
        case 3
          table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);       
        case 4
          table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          obstacle = RigidBodyBox([.1 .3 .2], [.55 0 1.0125], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);
      end
    end
    
    function robot = generateScene(options)      
      robot = Scenes.generateRobot(options);  
      world = robot.findLinkId('world');
      robot = Scenes.generateWorld(options, robot, world);
      robot = robot.compile();
    end
    
    function x = getFP(model, robot)
      %model can also be an fp file name
      switch model
        case 'v3'
          fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
        case 'v4'
          fp = load([getDrakePath(), '/../control/matlab/data/atlas_v4_fp.mat']);
          %                     fp = load([getDrakePath(), '/../valkyrie/matlab/atlas_v4_fp.mat']);
        case 'v5'
          fp = load([getDrakePath(), '/../control/matlab/data/atlas_v5_fp.mat']);
        case 'val'
          fp = load([getDrakePath(), '/../control/matlab/data/valkyrie_fp.mat']);
        otherwise
          fp = load([getDrakePath(), sprintf('/../control/matlab/data/%s.mat', model)]);
          %                 case 'lwr'
          %                     fp = load([getDrakePath(),'/../control/matlab/data/kuka_lwr_fp.mat']);
      end
      x = fp.xstar(1:robot.getNumPositions());
    end
    
    function targetObjectPos = getTargetObjPos(options)
      switch options.scene
        case {1, 2, 4}
          targetObjectPos = [0.8 0 1.0625];
        case 3
          if strcmp(options.model, 'val')
            targetObjectPos = [0.7 0 0.6];
          else
            targetObjectPos = [0.8 0 0.6];
          end
      end
    end
    
    function hand = getGraspingHand(options, robot)
      if strcmp(options.graspingHand, 'left')
        if strcmp(options.model, 'val')
          hand = robot.findLinkId('LeftPalm');
        else
          hand = robot.findLinkId('l_hand');
        end
      else
        if strcmp(options.model, 'val')
          hand = robot.findLinkId('RightPalm');
        else
          hand = robot.findLinkId('r_hand');
        end
      end
    end
    
    function hand = getNonGraspingHand(options, robot)
      if strcmp(options.graspingHand, 'left')
        if strcmp(options.model, 'val')
          hand = robot.findLinkId('RightPalm');
        else
          hand = robot.findLinkId('r_hand');
        end
      else
        if strcmp(options.model, 'val')
          hand = robot.findLinkId('LeftPalm');
        else
          hand = robot.findLinkId('l_hand');
        end
      end
    end
    
    function lFoot = getLeftFoot(options, robot)
      switch options.model
        case {'v3', 'v4', 'v5'}
          lFoot = robot.findLinkId('l_foot');
        case 'val'
          lFoot = robot.findLinkId('LeftUpperFoot');
      end
    end
    
    function rFoot = getRightFoot(options, robot)
      switch options.model
        case {'v3', 'v4', 'v5'}
          rFoot = robot.findLinkId('r_foot');
        case 'val'
          rFoot = robot.findLinkId('RightUpperFoot');
      end
    end
    
    function frame = getPointInLinkFrame(options)
      if strcmp(options.graspingHand, 'left')
        switch options.model
          case 'v3'
            frame = [0; 0.4; 0];
          case {'v4', 'v5'}
            frame = [0; -0.3; 0];
          case 'val'
            frame = [0.08; 0.0; -0.07];
        end
      else
        switch options.model
          case 'v3'
            frame = [0; -0.4; 0];
          case {'v4', 'v5'}
            frame = [0; -0.3; 0];
          case 'val'
            frame = [0.08; 0.0; 0.07];
        end
      end
    end
    
    function quasiStaticConstraint = addQuasiStaticConstraint(options, robot)
      if strcmp(options.model, 'val')
        l_foot = robot.findLinkId('LeftUpperFoot');
        r_foot = robot.findLinkId('RightUpperFoot');
      else
        l_foot = robot.findLinkId('l_foot');
        r_foot = robot.findLinkId('r_foot');
      end
      l_foot_pts = robot.getBody(l_foot).getTerrainContactPoints();
      r_foot_pts = robot.getBody(r_foot).getTerrainContactPoints();
      quasiStaticConstraint = QuasiStaticConstraint(robot, [-inf, inf], 1);
      quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
      quasiStaticConstraint = quasiStaticConstraint.setActive(true);
      quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
      quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);
    end
    
    function constraints = fixedFeetConstraints(options, robot, state, foot)
      if nargin < 4, foot = 'both'; end;
      if nargin < 3
        state = Scenes.getFP(options.model, robot);
      end
      leftConstraints = {};
      rightConstraints = {};
      kinsol = robot.doKinematics(state);
      if strcmp(options.model, 'val')
        l_foot = robot.findLinkId('LeftUpperFoot');
        r_foot = robot.findLinkId('RightUpperFoot');
      else
        l_foot = robot.findLinkId('l_foot');
        r_foot = robot.findLinkId('r_foot');
      end
      if any(strcmp(foot, {'both', 'left'}))
        footPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
        %                 drawLinkFrame(robot, l_foot, state, 'Left Foot Frame');
        leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
        leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);
        leftConstraints = {leftFootPosConstraint, leftFootQuatConstraint};
      end
      if any(strcmp(foot, {'both', 'right'}))
        footPose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
        %                 drawLinkFrame(robot, r_foot, state, 'Right Foot Frame');
        rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
        rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);
        rightConstraints = {rightFootPosConstraint, rightFootQuatConstraint};
      end
      constraints = [leftConstraints, rightConstraints];
    end
    
    function constraint = graspingForearmAlignConstraint(options, robot)
      if strcmp(options.graspingHand, 'left')
        side = [true false];
      else
        side = [false true];
      end
      rotation = [0; 0; 0];
      switch options.model
        case 'v3'
          names = {'l_farm', 'r_farm'};
        case 'v4'
          names = {'l_ufarm', 'r_ufarm'};
        case 'v5'
          if strcmp(options.graspingHand, 'left')
            rotation = [0; pi; 0];
          end
          names = {'l_ufarm', 'r_ufarm'};
        case 'val'
          if strcmp(options.graspingHand, 'left')
            rotation = [0; pi/2; -pi/2];
          else
            rotation = [0; -pi/2; -pi/2];
          end
          names = {'LeftForearm', 'RightForearm'};
      end
      farm = robot.findLinkId(names{side});
      hand = Scenes.getGraspingHand(options, robot);
      constraint = RelativeQuatConstraint(robot, hand, farm, rpy2quat(rotation), 0);
    end
    
    function constraint = nonGraspingHandPositionConstraint(options, robot)
      hand = Scenes.getNonGraspingHand(options, robot);
      if strcmp(options.graspingHand, 'left')
        position = [0.0; -0.5; 0.8];
      else
        position = [0.0; 0.5; 0.8];
      end
      if any(options.scene == [1 2])
        constraint = WorldPositionConstraint(robot, hand, [0; 0; 0], position, position);
      end
    end
    
    function constraint = nonGraspingHandDistanceConstraint(options, robot, dist)
      hand = Scenes.getNonGraspingHand(options, robot);
      model = strcmp(options.model,{'val', 'v4'});
      if strcmp(options.graspingHand, 'left')
        linkNames = {'RightLeg', 'r_uleg'};
        leg = robot.findLinkId(linkNames{model});
      else
        linkNames = {'RightLeg', 'r_uleg'};
        leg = robot.findLinkId(linkNames{model});
      end
      constraint = Point2PointDistanceConstraint(robot, hand, leg, [0; 0; 0], [0; 0; 0], dist, Inf);
    end
    
    function constraint = pelvisOffsetConstraint(options, robot, offset, state)
      if nargin < 3, offset = -0.2; end
      if nargin < 4
        state = Scenes.getFP(options.model, robot);
      end
      kinsol = robot.doKinematics(state);
      pelvis = robot.findLinkId('pelvis');
      pelvisPose = robot.forwardKin(kinsol,pelvis, [0; 0; 0], 2);
      constraint = WorldPositionConstraint(robot, pelvis, [0; 0; 0], pelvisPose(1:3) + [0; 0; offset], pelvisPose(1:3) + [0; 0; offset]);
    end
    
    function constraints = addGoalConstraint(options, robot)
      hand = Scenes.getGraspingHand(options, robot);
      goalFrame = [eye(3) Scenes.getTargetObjPos(options)'; 0 0 0 1];
      if strcmp(options.graspingHand, 'right')
        frameRotation = -1;
      else
        frameRotation = 1;
      end
      point_in_link_frame = Scenes.getPointInLinkFrame(options);
      lower_bounds = [0.0; 0.0; 0.0];
      upper_bounds = [0.0; 0.0; 0.0];
      switch options.model
        case 'v3'
          goalQuatConstraint = WorldQuatConstraint(robot, hand, axis2quat([1; 1; 1; frameRotation].*[0 0 1 -pi/2]'), 10*pi/180, [1.0, 1.0]);
          %                     rot_mat = [axis2rotmat([0 0 1 -pi/2]) [0; 0; 0]; [0 0 0 1]];
          %                     trans_mat = [eye(3) -point_in_link_frame; [0 0 0 1]];
        case 'v4'
          goalQuatConstraint = WorldQuatConstraint(robot, hand, rpy2quat([0 pi/2 pi/2]'), 10*pi/180, [1.0, 1.0]);
          %                     rot_mat = [rpy2rotmat([0 pi/2 pi/2]) [0; 0; 0]; [0 0 0 1]];
          %                     trans_mat = [eye(3) -point_in_link_frame; [0 0 0 1]];
        case 'v5'
          goalQuatConstraint = WorldQuatConstraint(robot, hand, rpy2quat([1; frameRotation; 1].*[0 -pi/2 pi/2]'), 10*pi/180, [1.0, 1.0]);
          %                     rot_mat = [rpy2rotmat([0 pi/2 pi/2]) [0; 0; 0]; [0 0 0 1]];
          %                     trans_mat = [eye(3) -point_in_link_frame; [0 0 0 1]];
        case 'val'
          goalQuatConstraint = WorldQuatConstraint(robot, hand, rpy2quat([-pi/2 0 0 ]'), 10*pi/180, [1.0, 1.0]);
          goalEulerConstraint = WorldEulerConstraint(robot, hand, [-pi/2;0; -pi], [-pi/2; 0; pi]);
      end
%       goalPosConstraint = WorldPositionInFrameConstraint(robot, hand, point_in_link_frame, goalFrame,...
%         lower_bounds, upper_bounds, [1.0, 1.0]);
      goalDistConstraint = Point2PointDistanceConstraint(robot, hand, robot.findLinkId('world'), point_in_link_frame, goalFrame(1:3, 4), 0, 0);
%       constraints = {goalPosConstraint, goalQuatConstraint};
      constraints = {goalDistConstraint, goalEulerConstraint};
      
      
      %             drawFrame(ref_frame, 'Goal Frame')
      %             drawFrame(ref_frame*rot_mat*trans_mat, 'Goal Offset Frame');
    end
    

    function constraints = generateEEConstraints(robot, options, x)
      constraints = {};
      xyz = x(1:3);
      quat = x(4:7);
      constraints{1} = WorldPositionConstraint(robot, Scenes.getGraspingHand(options, robot), Scenes.getPointInLinkFrame(options), xyz, xyz);
      constraints{2} = WorldQuatConstraint(robot, Scenes.getGraspingHand(options, robot), quat, 10*pi/180);
    end
    
  end
  
end

