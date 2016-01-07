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
    
    function [robot, urdf] = generateRobot(options)
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
    
    function [robot, urdf] = generateScene(options)
      [robot, urdf] = Scenes.generateRobot(options);
      world = robot.findLinkId('world');
      robot = Scenes.generateWorld(options, robot, world);
      robot = robot.compile();
    end
    
    function points = getOctomap(options)
      switch options.scene
        case {1, 2, 3, 4, 5}
          om_file = sprintf('scene%d', options.scene);
        case {6, 7, 8, 9}
          om_file = 'scene6-9';
      end
      om_file = fullfile(getDrakePath(),'../../../../drc-testing-data/final_pose_planner', om_file);
      points = load(om_file);
      points = points.points;
    end
    
    function visualizeOctomap(options)
      points = Scenes.getOctomap(options);
      lcmClient = LCMGLClient('point cloud');
      lcmClient.glPointSize(10)
      lcmClient.points(points(1,:), points(2,:), points(3,:))
      lcmClient.switchBuffers();
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
    
    function hand = getHand(options, robot, side)
      hand.v3.left = 'l_hand';
      hand.v4.left = 'l_hand';
      hand.v5.left = 'l_hand';
      hand.val1.left = 'LeftPalm';
      hand.val2.left = 'leftPalm';
      hand.v3.right = 'r_hand';
      hand.v4.right = 'r_hand';
      hand.v5.right = 'r_hand';
      hand.val1.right = 'RightPalm';
      hand.val2.right = 'rightPalm';
      hand = robot.findLinkId(hand.(options.model).(side));
    end
    
    function hand = getGraspingHand(options, robot)
      hand = Scenes.getHand(options, robot, options.graspingHand);
    end
    
    function hand = getNonGraspingHand(options, robot)
      if strcmp(options.graspingHand, 'left')
        hand = Scenes.getHand(options, robot, 'right');
      else
        hand = Scenes.getHand(options, robot, 'left');
      end
    end
    
    function l_foot = getLeftFoot(options, robot)
      l_foot.v3 = 'l_foot';
      l_foot.v4 = 'l_foot';
      l_foot.v5 = 'l_foot';
      l_foot.val1 = 'LeftUpperFoot';
      l_foot.val2 = 'LeftFoot';
      l_foot = robot.findLinkId(l_foot.(options.model));
    end
    
    function r_foot = getRightFoot(options, robot)
      r_foot.v3 = 'r_foot';
      r_foot.v4 = 'r_foot';
      r_foot.v5 = 'r_foot';
      r_foot.val1 = 'RightUpperFoot';
      r_foot.val2 = 'RightFoot';
      r_foot = robot.findLinkId(r_foot.(options.model));
    end
    
    function base = getBase(options, robot)
      base.v3 = 'pelvis';
      base.v4 = 'pelvis';
      base.v5 = 'pelvis';
      base.val1 = 'Pelvis';
      base.val2 = 'pelvis';
      base = robot.findLinkId(base.(options.model));
    end
    
    function joint_idx = getBackJoints(options, robot)
      names.v3 = {'back_bkz', 'back_bky', 'back_bkx'};
      names.v4 = {'back_bkz', 'back_bky', 'back_bkx'};
      names.v5 = {'back_bkz', 'back_bky', 'back_bkx'};
      names.val1 = {'WaistRotator', 'WaistExtensor', 'WaistLateralExtensor'};
      names.val2 = {'torsoYaw', 'torsoPitch', 'torsoRoll'};
      joint_idx = cellfun(@robot.findPositionIndices, names.(options.model)');
    end
    
    function forearm = getForearm(options, robot, side)
      forearm.v3.left = 'l_farm';
      forearm.v4.left = 'l_ufarm';
      forearm.v5.left = 'l_ufarm';
      forearm.val1.left = 'LeftForearm';
      forearm.val2.left = 'leftForearmLink';
      forearm.v3.right = 'r_farm';
      forearm.v4.right = 'r_ufarm';
      forearm.v5.right = 'r_ufarm';
      forearm.val1.right = 'RightForearm';
      forearm.val2.right = 'rightForearmLink';
      forearm = robot.findLinkId(forearm.(options.model).(side));
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
    
    function quasiStaticConstraint = generateQuasiStaticConstraint(options, robot)
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
    
    function constraints = generateFeetConstraints(options, robot, state)
      if nargin < 3
        state = Scenes.getFP(options.model, robot);
      end
      l_foot = Scenes.getLeftFoot(options, robot);
      r_foot = Scenes.getRightFoot(options, robot);
      kinsol = robot.doKinematics(state);
      l_pose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
      r_pose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
      lb.fixed = [0; 0; 0];
      lb.sliding = [-inf; -inf; 0];
      ub.fixed = [0; 0; 0];
      ub.sliding = [inf; inf; 0];
      l_pos_const = WorldPositionConstraint(robot, l_foot, [0; 0; 0], l_pose(1:3) + lb.(options.feet_constraint), l_pose(1:3) + ub.(options.feet_constraint));
      l_quat_const = WorldQuatConstraint(robot, l_foot, l_pose(4:7), 0.0);
      r_pos_const = WorldPositionConstraint(robot, r_foot, [0; 0; 0], r_pose(1:3) + lb.(options.feet_constraint), r_pose(1:3) + ub.(options.feet_constraint));
      r_quat_const = WorldQuatConstraint(robot, r_foot, r_pose(4:7), 0.0);
      rel_const = RelativePositionConstraint(robot, [0;0;0], l_pose(1:3) - r_pose(1:3), l_pose(1:3) - r_pose(1:3), l_foot, r_foot);
      constraints = {l_pos_const, l_quat_const, r_pos_const, r_quat_const, rel_const};
    end
    
    function constraint = graspingForearmAlignConstraint(options, robot)
      grHand = options.graspingHand;
      forearm = Scenes.getForearm(options, robot, grHand);
      switch options.model
        case 'v5'
          rotation.right = [0; 0; 0];
          rotation.left = [0; pi; 0];
        case 'val1'
          rotation.right = [0; -pi/2; -pi/2];
          rotation.left = [0; pi/2; -pi/2];
        case 'val2'
          rotation.right = [0; -pi/2; -pi/2];
          rotation.left = [0; pi/2; -pi/2];
      end
      hand = Scenes.getGraspingHand(options, robot);
      constraint = RelativeQuatConstraint(robot, hand, forearm, rpy2quat(rotation.(grHand)), 0);
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
      pelvis = Scenes.getBase(options, robot);
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
    
    function constraint = generateBackConstraint(options, robot, q_start)
      joint_idx = Scenes.getBackJoints(options, robot);
      lb.free = q_start(joint_idx) + deg2rad([-15; -5; -inf]);
      lb.limited = q_start(joint_idx) + deg2rad([-5; -5; -inf]);
      lb.fixed = q_start(joint_idx) + deg2rad([0; 0; 0]);
      ub.free = q_start(joint_idx) + deg2rad([15; 25; inf]);
      ub.limited = q_start(joint_idx) + deg2rad([5; 5; inf]);
      ub.fixed = q_start(joint_idx) + deg2rad([0; 0; 0]);
      
      constraint = PostureConstraint(robot);
      constraint = constraint.setJointLimits(joint_idx, lb.(options.back_constraint), ub.(options.back_constraint));
    end
    
    function constraint = generateBaseConstraint(options, robot, q_start)
      joint_idx = robot.body(Scenes.getBase(options, robot)).position_num;
      lb.free = q_start(joint_idx) + [-inf; -inf; -inf; -pi; -pi; -pi];
      lb.limited = q_start(joint_idx) + [[-inf; -inf; -inf]; deg2rad([-5; -5; -inf])];
      lb.fixed = q_start(joint_idx) + [0; 0; 0; 0; 0; 0];
      lb.xyz = q_start(joint_idx) + [-inf; -inf; -inf; 0; 0; 0];
      lb.z = q_start(joint_idx) + [0; 0; -inf; 0; 0; 0];
      ub.free = q_start(joint_idx) + [inf; inf; inf; pi; pi; pi];
      ub.limited = q_start(joint_idx) + [[inf; inf; inf]; deg2rad([5; 15; inf])];
      ub.fixed = q_start(joint_idx) + [0; 0; 0; 0; 0; 0];
      ub.xyz = q_start(joint_idx) + [inf; inf; inf; 0; 0; 0];
      ub.z = q_start(joint_idx) + [0; 0; inf; 0; 0; 0];
      
      constraint = PostureConstraint(robot);
      constraint = constraint.setJointLimits(joint_idx, lb.(options.base_constraint), ub.(options.base_constraint));
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

