classdef Scenes
  
  methods(Static)
    
    function names = getSceneNames(scenes)
      names = {'scene1', 'scene2', 'scene3'};
      if nargin == 1
        names = names{scenes};
      end
    end
    
    function nScenes = getnScenes()
      nScenes = 9;
    end
    
    function robot = generateRobot(options)
      warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      warning('off','Drake:CollisionFilterGroup:DiscardingCollisionFilteringInfo');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
      switch options.model
%         case 'v3'
%           if options.convex_hull
%             urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_chull.urdf');
%           else
%             urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_full.urdf');
%           end
%         case 'v4'
%           if options.convex_hull
%             urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_chull.urdf');
%           else
%             urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_full.urdf');
%           end
        case {'v3', 'v4'}
          error('Version 3 and 4 of the Atlas Robot are no longer supported. Use version 5 instead.')
        case 'v5'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'../../','models','atlas_v5','model_chull.urdf');
          else
            urdf = fullfile(getDrakePath(),'../../','models','atlas_v5','model_full.urdf');
          end
        case 'val1'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'../../','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
          else
            urdf = fullfile(getDrakePath(),'../../','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
          end
        case 'val2'
          if options.convex_hull
            urdf = fullfile(getDrakePath(),'../../','models','val_description/urdf','valkyrie_sim_drake.urdf');
          else
            urdf = fullfile(getDrakePath(),'../../','models','val_description/urdf','model.urdf');
          end
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
          table = RigidBodyBox([.6 1 .025], [.7 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);          
        case 2
          table = RigidBodyBox([.6 1 .025], [.7 0 .9], [0 0 0]);
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
          table = RigidBodyBox([.6 1 .025], [.7 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);       
        case 4
          table = RigidBodyBox([1 1 .025], [.7 0 .9], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          obstacle = RigidBodyBox([.1 .2 .2], [.25 0 1.0125], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);
        case 5
          table = RigidBodyBox([.5 1 .05], [.65 0 .925], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          leg1 = RigidBodyBox([.5 .05 .9], [.65 .5 .45], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, leg1);
          
          leg2 = RigidBodyBox([.5 .05 .9], [.65 -.5 .45], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, leg2);
          
          obstacle1 = RigidBodyBox([.1 .3 .05], [.25 0.35 .96], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle1);
          
          obstacle2 = RigidBodyBox([.1 .3 .05], [.25 -0.35 .96], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle2);
          
          obstacle3 = RigidBodyBox([.2 1 .05], [.35 0 1.31], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, obstacle3);
          
          targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(options), [0 0 0]);
          targetObject = targetObject.setColor([1 0 0]);
          robot = addGeometryToBody(robot, world_link, targetObject);
        case {6, 7, 8, 9}
          table = RigidBodyBox([.5 1 .05], [.65 0 .825], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, table);
          
          leg1 = RigidBodyBox([.5 .05 .8], [.65 .5 .4], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, leg1);
          
          leg2 = RigidBodyBox([.5 .05 .8], [.65 -.5 .4], [0 0 0]);
          robot = addGeometryToBody(robot, world_link, leg2);
          
          goals = {};
          
          goals{1} = RigidBodyCylinder(.045, .22, [.5 -.2 .96], [0 0 0]);
          goals{2} = RigidBodyCylinder(.035, .22, [.7 -.1 .96], [0 0 0]);
          goals{3} = RigidBodyBox([.07 .07 .25], [.7 .1 .975], [0 0 0]);
          goals{4} = RigidBodyBox([.08 .08 .24], [.5 .3 .97], [0 0 0]);
          
          goals{options.scene-5} = goals{options.scene-5}.setColor([1 0 0]);
          
          robot = addGeometryToBody(robot, world_link, goals{1});
          robot = addGeometryToBody(robot, world_link, goals{2});
          robot = addGeometryToBody(robot, world_link, goals{3});
          robot = addGeometryToBody(robot, world_link, goals{4});
      end
    end
    
    function robot = generateScene(options)
      robot = Scenes.generateRobot(options);
      world = robot.findLinkId('world');
      robot = Scenes.generateWorld(options, robot, world);
      robot = robot.compile();
    end
    
    function robot = graspObject(T, options)      
      object = RigidBodyCylinder(.045, .19, T);
      robot = Scenes.generateRobot(options);  
      hand = Scenes.getGraspingHand(options, robot);
      robot = addGeometryToBody(robot, hand, object);
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
          fp = load([getDrakePath(), '/../../control/matlab/data/atlas_v4/atlas_v4_fp.mat']);
        case 'v5'
          fp = load([getDrakePath(), '/../../control/matlab/data/atlas_v5/atlas_v5_fp.mat']);
        case 'val1'
          fp = load([getDrakePath(), '/../../control/matlab/data/valkyrie/valkyrie_fp.mat']);
        case 'val2'
          fp = load([getDrakePath(), '/../../control/matlab/data/val_description/valkyrie_fp_june2015.mat']);
        otherwise
          fp = load([getDrakePath(), sprintf('/../../control/matlab/data/%s.mat', model)]);
          %                 case 'lwr'
          %                     fp = load([getDrakePath(),'/../control/matlab/data/kuka/kuka_lwr_fp.mat']);
      end
      x = fp.xstar(1:robot.getNumPositions());
    end
    
    function targetObjectPos = getTargetObjPos(options)
      switch options.scene
        case {1, 2}
          targetObjectPos = [0.8 0 1.0625];
        case 3
          if any(strcmp(options.model, {'val1', 'val2'}))
            targetObjectPos = [0.7 0 0.6];
          else
            targetObjectPos = [0.8 0 0.6];
          end
        case 4
          targetObjectPos = [0.5 0 1.0625];
        case 5
          targetObjectPos = [0.65 0 1.1];
        case 6
          targetObjectPos = [.5 -.2 .96];
        case 7
          targetObjectPos = [.7 -.1 .96];
        case 8
          targetObjectPos = [.7 .1 .975];
        case 9
          targetObjectPos = [.5 .3 .97];
      end
    end
    
    function ee_pose = getDesiredEePose(options)
      ee_pose.val2(1).right = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 pi/2])];
      ee_pose.val2(1).left = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 -pi/2])];
      ee_pose.val2(2) = ee_pose.val2(1);
      ee_pose.val2(3) = ee_pose.val2(1);
      ee_pose.val2(4).right = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 3*pi/4])];
      ee_pose.val2(4).left = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 -3*pi/4])];
      ee_pose.val2(5) = ee_pose.val2(1);
      ee_pose.val2(6) = ee_pose.val2(1);
      ee_pose.val2(7).right = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 pi/4])];
      ee_pose.val2(7).left = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 -pi/2])];
      ee_pose.val2(8) = ee_pose.val2(1);
      ee_pose.val2(9) = ee_pose.val2(1);
      ee_pose = ee_pose.(options.model)(options.scene).(options.graspingHand);
    end
    
    function hand = getGraspingHand(options, robot)
      if strcmp(options.graspingHand, 'left')
        if any(strcmp(options.model, {'val1', 'val2'}))
          hand = robot.findLinkId('LeftPalm');
        else
          hand = robot.findLinkId('l_hand');
        end
      else
        if any(strcmp(options.model, {'val1', 'val2'}))
          hand = robot.findLinkId('RightPalm');
        else
          hand = robot.findLinkId('r_hand');
        end
      end
    end
    
    function hand = getNonGraspingHand(options, robot)
      if strcmp(options.graspingHand, 'left')
        if any(strcmp(options.model, {'val1', 'val2'}))
          hand = robot.findLinkId('RightPalm');
        else
          hand = robot.findLinkId('r_hand');
        end
      else
        if any(strcmp(options.model, {'val1', 'val2'}))
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
        case 'val1'
          lFoot = robot.findLinkId('LeftUpperFoot');
        case 'val2'
          lFoot = robot.findLinkId('LeftFoot');
      end
    end
    
    function rFoot = getRightFoot(options, robot)
      switch options.model
        case {'v3', 'v4', 'v5'}
          rFoot = robot.findLinkId('r_foot');
        case 'val1'
          rFoot = robot.findLinkId('RightUpperFoot');
        case 'val2'
          rFoot = robot.findLinkId('RightFoot');
      end
    end
%NOTEST

    function frame = getPointInLinkFrame(options)
      frame.v3.left = [0; 0.4; 0];
      frame.v4.left = [0; -0.3; 0];
      frame.v5.left = [0; -0.3; 0];
      frame.val1.left = [0.08; 0.0; -0.07];
      frame.val2.left = [0.08; 0.07; 0];
      frame.v3.right = [0; -0.4; 0];
      frame.v4.right = [0; -0.3; 0];
      frame.v5.right = [0; -0.3; 0];
      frame.val1.right = [0.08; 0.0; -0.07];
      frame.val2.right = [0.08; -0.07; 0];
      frame = frame.(options.model).(options.graspingHand);
    end
    
    function quasiStaticConstraint = addQuasiStaticConstraint(options, robot)
      l_foot = Scenes.getLeftFoot(options, robot);
      r_foot = Scenes.getRightFoot(options, robot);
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
      l_foot = Scenes.getLeftFoot(options, robot);
      r_foot = Scenes.getRightFoot(options, robot);
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
    
    function constraints = slidingFeetConstraints(options, robot, state)
      if nargin < 3
        state = Scenes.getFP(options.model, robot);
      end
      leftConstraints = {};
      rightConstraints = {};
      kinsol = robot.doKinematics(state);
      l_foot = Scenes.getLeftFoot(options, robot);
      r_foot = Scenes.getRightFoot(options, robot);
      l_footPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
      %                 drawLinkFrame(robot, l_foot, state, 'Left Foot Frame');
      leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, [0; 0; 0], [-inf; -inf; l_footPose(3)], [inf; inf; l_footPose(3)]);
      leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, l_footPose(4:7), 0.0, [0.0, 1.0]);
      leftConstraints = {leftFootPosConstraint, leftFootQuatConstraint};
      r_footPose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
      %                 drawLinkFrame(robot, r_foot, state, 'Right Foot Frame');
      rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, [0; 0; 0], [-inf; -inf; r_footPose(3)], [inf; inf; r_footPose(3)]);
      rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, r_footPose(4:7), 0.0, [0.0, 1.0]);
      rightConstraints = {rightFootPosConstraint, rightFootQuatConstraint};
      relativeConstraint = RelativePositionConstraint(robot, [0;0;0], l_footPose(1:3) - r_footPose(1:3), l_footPose(1:3) - r_footPose(1:3), l_foot, r_foot);
      constraints = [leftConstraints, rightConstraints, {relativeConstraint}];
    end
    
    function constraint = graspingForearmAlignConstraint(options, robot)
      grHand = options.graspingHand;
      switch options.model
        case 'v3'
          name.right = 'r_farm';
          name.left = 'l_farm';
        case 'v4'
          name.right = 'r_ufarm';
          name.left = 'l_ufarm';
        case 'v5'
          rotation.right = [0; 0; 0];
          rotation.left = [0; pi; 0];
          name.right = 'r_ufarm';
          name.left = 'l_ufarm';
        case 'val1'
          rotation.right = [0; -pi/2; -pi/2];
          rotation.left = [0; pi/2; -pi/2];
          name.right = 'RightForearm';
          name.left = 'LeftForearm';
        case 'val2'
          rotation.right = [0; -pi/2; -pi/2];
          rotation.left = [0; pi/2; -pi/2];
          name.right = 'RightForearmLink';
          name.left = 'LeftForearmLink';
      end
      hand = Scenes.getGraspingHand(options, robot);
      constraint = RelativeQuatConstraint(robot, hand, name.(grHand), rpy2quat(rotation.(grHand)), 0);
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
      trunk.val1 = 'Trunk';
      trunk.val2 = 'Torso';
      trunk.v4 = 'utorso';
      trunk.v5 = 'utorso';
      elbow.left.val1 = 'RightElbowExtensor';
      elbow.left.val2 = 'RightElbowPitchLink';
      elbow.left.v4 = 'r_larm';
      elbow.left.v5 = 'r_larm';
      elbow.right.val1 = 'LeftElbowExtensor';
      elbow.right.val2 = 'LeftElbowPitchLink';
      elbow.right.v4 = 'l_larm';
      elbow.right.v5 = 'l_larm';
      elbow = robot.findLinkId(elbow.(options.graspingHand).(options.model));
      trunk = robot.findLinkId(trunk.(options.model));
      constraint = Point2PointDistanceConstraint(robot, elbow, trunk, [0; 0; 0], [0; 0; 0], dist, Inf);
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
      point_in_link_frame = Scenes.getPointInLinkFrame(options);
      goalFrame = [eye(3) Scenes.getTargetObjPos(options)'; 0 0 0 1];
      goalEulerConstraint.left.v5 = WorldEulerConstraint(robot, hand, [-pi; pi/2;0 ], [pi; pi/2; 0]);
      goalEulerConstraint.right.v5 = WorldEulerConstraint(robot, hand, [-pi; -pi/2;0 ], [pi; -pi/2; 0]);
      goalEulerConstraint.left.val1 = WorldEulerConstraint(robot, hand, [-pi/2;0; -pi], [-pi/2; 0; pi]);
      goalEulerConstraint.right.val1 = goalEulerConstraint.left.val1;
      goalEulerConstraint.left.val2 = WorldEulerConstraint(robot, hand, [0;0; -pi], [0; 0; pi]);
      goalEulerConstraint.right.val2 = goalEulerConstraint.left.val2;
      goalDistConstraint = Point2PointDistanceConstraint(robot, hand, robot.findLinkId('world'), point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
      constraints = {goalDistConstraint, goalEulerConstraint.(options.graspingHand).(options.model)}; 
%       drawFrame(ref_frame, 'Goal Frame')
%       drawFrame(ref_frame*rot_mat*trans_mat, 'Goal Offset Frame');
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

