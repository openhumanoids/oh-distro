classdef Scenes
    properties
        sceneNames
        nScenes
    end
    
    methods
        function obj = Scenes()
            obj.sceneNames = {'scene1', 'scene2', 'scene3'};
            obj.nScenes = length(obj.sceneNames);
        end
    end
        
    methods(Static)
        function robot = generate(options, robot, world_link)
            scene = options.scene;
            switch scene
                case 'debris'
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
                case 'scene1'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);
                    
                    targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(scene), [0 0 0]);
                    targetObject = targetObject.setColor([1 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
                case 'scene2'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);

                    obstacle = RigidBodyBox([.3 .3 .4], [.55 .35 1.1125], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    obstacle = RigidBodyBox([.3 .3 .4], [.55 -0.35 1.1125], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    obstacle = RigidBodyBox([.3 1 .4], [.55 0 1.5125], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(scene), [0 0 0]);
                    targetObject = targetObject.setColor([1 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
                case 'scene3'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);

                    targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(scene), [0 0 0]);
                    targetObject = targetObject.setColor([1 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
            end
        end
        
        function fp = getFP(model)
            switch model
                case 'v3' 
                    fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
                case 'v4' 
                    fp = load([getDrakePath(), '/../control/matlab/data/atlas_v4_fp.mat']);
%                     fp = load([getDrakePath(), '/../valkyrie/matlab/atlas_v4_fp.mat']);
                case 'v5' 
                    fp = load([getDrakePath(), '/../control/matlab/data/atlas_v5_fp.mat']);
                case 'lwr'
                    fp = load([getDrakePath(),'/../control/matlab/data/kuka_lwr_fp.mat']);
%                 case 'val' 
%                     fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
                otherwise
                    fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
            end                    
        end
        
        function targetObjectPos = getTargetObjPos(scene)
            switch scene
                case 'scene1'
                    targetObjectPos = [0.8 0 1.0625];
                case 'scene2'
                    targetObjectPos = [0.8 0 1.0625];
                case 'scene3'
                    targetObjectPos = [0.8 0 0.6];
            end
        end
        
        function quasiStaticConstraint = addQuasiStaticConstraint(robot)
            l_foot = robot.findLinkId('l_foot');
            r_foot = robot.findLinkId('r_foot');
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
                fp = Scenes.getFP(options.model);
                state = fp.xstar(1:robot.getNumPositions());
            end
            constraints = {};
            kinsol = robot.doKinematics(state);
            if any(strcmp(foot, {'both', 'left'}))
                l_foot = robot.findLinkId('l_foot');
                footPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
                leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, -footPose(1:3) , [0; 0; 0], [0; 0; 0], [0.0, 1.0]);            
                leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);
                constraints = [constraints, {leftFootPosConstraint, leftFootQuatConstraint}];
            end
            if any(strcmp(foot, {'both', 'right'}))
                r_foot = robot.findLinkId('r_foot');
                footPose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
                rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, -footPose(1:3) , [0; 0; 0], [0; 0; 0], [0.0, 1.0]);            
                rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);
                constraints = [constraints, {rightFootPosConstraint, rightFootQuatConstraint}];
            end
        end
        
        function constraint = pelvisHeightConstraint(options, robot, offset, state)
            if nargin < 4
                fp = Scenes.getFP(options.model);
                state = fp.xstar(1:robot.getNumPositions());
            end
            if nargin < 3, offset = 0; end;
            kinsol = robot.doKinematics(state);
            pelvis = robot.findLinkId('pelvis');
            pelvisPose = robot.forwardKin(kinsol,pelvis, [0; 0; 0], 2);
            constraint = WorldPositionConstraint(robot, pelvis, -pelvisPose(1:3) - [0; 0; offset] , [0; 0; 0], [0; 0; 0], [0.0, 1.0]);             
        end
        
        function constraints = addGoalConstraint(options, robot)
            Scenes.getTargetObjPos(options.scene);
            ref_frame = [eye(3) Scenes.getTargetObjPos(options.scene)'; 0 0 0 1];
            l_hand = robot.findLinkId('l_hand');        
            lower_bounds = [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0];
            switch options.model
                case 'v4'
                    point_in_link_frame = [0; 0.3; 0]; 
                    goalQuatConstraint = WorldQuatConstraint(robot, l_hand, rpy2quat([0 pi/2 -pi/2]'), 10*pi/180, [1.0, 1.0]);
                case 'v3'
                    point_in_link_frame = [0; 0.4; 0]; 
                    goalQuatConstraint = WorldQuatConstraint(robot, l_hand, axis2quat([0 0 1 -pi/2]'), 10*pi/180, [1.0, 1.0]);
                otherwise
                    point_in_link_frame = [0; 0.3; 0]; 
                    goalQuatConstraint = WorldQuatConstraint(robot, l_hand, rpy2quat([0 pi/2 -pi/2]'), 10*pi/180, [1.0, 1.0]);
            end
            goalPosConstraint = WorldPositionInFrameConstraint(robot, l_hand, point_in_link_frame, ref_frame,...
                                                           lower_bounds, upper_bounds, [1.0, 1.0]);
            constraints = {goalPosConstraint, goalQuatConstraint};
        end
    end
        
end
