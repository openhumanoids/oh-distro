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
                    robot = addGeometryToBody(robot, world_link, targetObject);
                case 'scene2'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);

                    obstacle = RigidBodyBox([.3 .3 .4], [.55 .35 1.10], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    obstacle = RigidBodyBox([.3 .3 .4], [.55 -0.35 1.10], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    obstacle = RigidBodyBox([.3 1 .4], [.55 0 1.50], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, obstacle);

                    targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(scene), [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
                case 'scene3'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);

                    targetObject = RigidBodyBox([.05 .05 .3], Scenes.getTargetObjPos(scene), [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
            end
        end
        
        function fp = getFP(robotVersion)
            switch robotVersion
                case 'v3' 
                    fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
                case 'v4' 
                    fp = load([getDrakePath(), '/examples/Atlas/data/atlas_v4_fp.mat']);
                case 'v5' 
                    fp = load([getDrakePath(), '/examples/Atlas/data/atlas_v5_fp.mat']);
%                 case 'val' 
%                     fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
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
        
        function nominalPoseConstraints = setNominalPoseConstraints(options, robot)
            scene = options.scene;
            quasiStaticConstraint = Scenes.addQuasiStaticConstraint(robot);
            [leftFootPosConstraint, leftFootQuatConstraint] = Scenes.addLeftFootConstraint(options, robot);
            [rightFootPosConstraint , rightFootQuatConstraint] = Scenes.addRightFootConstraint(options, robot);
            backConstraint = Scenes.addBackConstraint(robot);
            baseConstraint = Scenes.addBaseConstraint(options, robot);
            rightArmConstraint = Scenes.addRightArmConstraint(options, robot);
            leftLegConstraint = Scenes.addLeftLegConstraint(robot);
            switch scene
                case 'debris'
                    l_hand = robot.findLinkId('l_hand');
                    xyz_quat_start = [0.5969; -0.1587; 0.9; -0.2139; 0.6724; 0.3071; -0.6387];
                    l_hand_initial_position_constraint = WorldPositionConstraint(robot, l_hand, [0;0;0], xyz_quat_start(1:3), xyz_quat_start(1:3));
                    l_hand_initial_quat_constraint = WorldQuatConstraint(robot, l_hand, xyz_quat_start(4:7), 0*pi/180);
                    
                    nominalPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, backConstraint, baseConstraint, rightArmConstraint,leftLegConstraint,...
                                            l_hand_initial_position_constraint , l_hand_initial_quat_constraint};                            
                otherwise
                    nominalPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, backConstraint, baseConstraint, rightArmConstraint,leftLegConstraint};
            end
        end
        
        function startPoseConstraints = setStartPoseConstraints(options, robot)
            leftArmConstraint = Scenes.addLeftArmConstraint(options, robot);
            nominalPoseConstraints = Scenes.setNominalPoseConstraints(options, robot);
            startPoseConstraints = [nominalPoseConstraints, {leftArmConstraint}];
        end
        
        function initialPoseConstraints = setInitialPoseConstraints(options, robot)            
            quasiStaticConstraint = Scenes.addQuasiStaticConstraint(robot);
            [leftFootPosConstraint, leftFootQuatConstraint] = Scenes.addLeftFootConstraint(options, robot);
            [rightFootPosConstraint , rightFootQuatConstraint] = Scenes.addRightFootConstraint(options, robot);
            initialPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint, rightFootQuatConstraint};
        end
        
        function endPoseConstraints = setEndPoseConstraints(options, robot)            
            scene = options.scene;
            quasiStaticConstraint = Scenes.addQuasiStaticConstraint(robot);
            [leftFootPosConstraint, leftFootQuatConstraint] = Scenes.addLeftFootConstraint(options, robot);
            [rightFootPosConstraint , rightFootQuatConstraint] = Scenes.addRightFootConstraint(options, robot);
            %backConstraint = Scenes.addBackConstraint(robot);            
            rightArmConstraint = Scenes.addRightArmConstraint(options, robot);
            [goalPosConstraint, goalQuatConstraint] = Scenes.addGoalConstraint(options, robot);
            %leftLegConstraint = Scenes.addLeftLegConstraint(robot);
            switch scene
                case 'debris'
                    l_hand = robot.findLinkId('l_hand');
                    xyz_quat_start = [0.5969; -0.1587; 0.9; -0.2139; 0.6724; 0.3071; -0.6387];
                    l_hand_initial_position_constraint = WorldPositionConstraint(robot, l_hand, [0;0;0], xyz_quat_start(1:3), xyz_quat_start(1:3));
                    l_hand_initial_quat_constraint = WorldQuatConstraint(robot, l_hand, xyz_quat_start(4:7), 0*pi/180);
                    
                    endPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, backConstraint, baseConstraint, rightArmConstraint,leftLegConstraint,...
                                            l_hand_initial_position_constraint , l_hand_initial_quat_constraint};                            
                otherwise
                    endPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, rightArmConstraint, goalPosConstraint, goalQuatConstraint};%, backConstraint, baseConstraint, leftLegConstraint};
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
        
        function [leftFootPosConstraint, leftFootQuatConstraint] = addLeftFootConstraint(options, robot)  
            l_foot = robot.findLinkId('l_foot');          
            point_in_link_frame = [0.0; 0.0; 0.0];            
            switch options.model
                case 'v3'
                    ref_frame = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341;...
                                 -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892;...
                                 -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311;...
                                 0.0, 0.0, 0.0, 1.0];
                    quat = [0.99999680768841015; -0.0024891132733300568; 0.00043417407699420605; -2.0517608182535892e-05];
                case 'v4'
                    ref_frame = [1.0, 2.0886515360949906e-18, -2.7750225355747022e-17, -0.0241133589390799;
                                 -1.9498748793552056e-18, 0.9999875000260416, 0.004999979166700381, 0.15477434310170868;
                                 2.776032169281971e-17, -0.004999979166700381, 0.9999875000260416, 0.07610619080870928;
                                 0.0, 0.0, 0.0, 1.0];
                    quat = [-9.99996875e-01; 2.49999740e-03; 1.38776801e-17; 1.00963476e-18];
            end
            lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            ref_frame = inv(ref_frame);
            leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);
            
            leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, quat, 0.0, [0.0, 1.0]);
        end
        
        function [rightFootPosConstraint, rightFootQuatConstraint] = addRightFootConstraint(options, robot)  
            r_foot = robot.findLinkId('r_foot');
            point_in_link_frame = [0.0; 0.0; 0.0];            
            switch options.model
                case 'v3'
                    ref_frame = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833;...
                                 4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941;...
                                 -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948;...
                                 0.0, 0.0, 0.0, 1.0];
                    quat = [0.99999684531339206; 0.0024841562616134435; 0.00037137837375452614; 2.0224619435999976e-05];
                case 'v4'
                    ref_frame = [1.00000000e+00, -3.82337502e-18, -2.77458885e-17 , -0.02411336;...
                                 3.68459836e-18,   9.99987500e-01,  -4.99997918e-03, -0.15477434;...
                                 2.77646585e-17,   4.99997918e-03,   9.99987500e-01, 0.07610619;...
                                 0.0, 0.0, 0.0, 1.0];
                    quat = [9.99996875e-01; 2.49999740e-03; -1.38776801e-17; 1.87699921e-18];
            end                    
            lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            ref_frame = inv(ref_frame);
            rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


            rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, quat, 0.0, [0.0, 1.0]);
        end
        
        function backConstraint = addBackConstraint(robot)
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');
            backConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
            joints_lower_limit = [-0.08726646259971647; -0.08726646259971647; -inf];
            joints_upper_limit = [0.08726646259971647; 0.08726646259971647; inf];
            backConstraint = backConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function baseConstraint = addBaseConstraint(options, robot)
            S = Scenes.getFP(options.model);
            q_nom = S.xstar(1:robot.getNumPositions());
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');
            baseConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
            joints_lower_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
            joints_upper_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
            baseConstraint = baseConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function rightArmConstraint = addRightArmConstraint(options, robot)
            S = Scenes.getFP(options.model);
            q_nom = S.xstar(1:robot.getNumPositions());   
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');     
            rightArmConstraint = PostureConstraint(robot, [-inf, inf]);
            switch options.model
                case 'v3'
                    joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
                case 'v4'
                    joint_inds = [joints.r_arm_shz; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
            end
            joints_lower_limit = q_nom(joint_inds);
            joints_upper_limit = q_nom(joint_inds);
            rightArmConstraint = rightArmConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function [goalPosConstraint, goalQuatConstraint] = addGoalConstraint(options, robot)
            Scenes.getTargetObjPos(options.scene);
            ref_frame = [eye(3) Scenes.getTargetObjPos(options.scene)'; 0 0 0 1];
            point_in_link_frame = [0; 0.4; 0]; 
            l_hand = robot.findLinkId('l_hand');        
            lower_bounds = [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0];
            goalPosConstraint = WorldPositionInFrameConstraint(robot, l_hand, point_in_link_frame, ref_frame,...
                                                                   lower_bounds, upper_bounds, [1.0, 1.0]);
            goalQuatConstraint = WorldQuatConstraint(robot, l_hand, axis2quat([0 0 1 -pi/2]'), 10*pi/180, [1.0, 1.0]);
        end
        
        function leftArmConstraint = addLeftArmConstraint(options, robot)
            S = Scenes.getFP(options.model);
            q_nom = S.xstar(1:robot.getNumPositions());   
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');     
            leftArmConstraint = PostureConstraint(robot, [0, 0]);
            switch options.model
                case 'v3'
                    joint_inds = [joints.l_arm_usy; joints.l_arm_shx; joints.l_arm_ely; joints.l_arm_elx; joints.l_arm_uwy; joints.l_arm_mwx; joints.neck_ay];
                case 'v4'
                    joint_inds = [joints.l_arm_shz; joints.l_arm_shx; joints.l_arm_ely; joints.l_arm_elx; joints.l_arm_uwy; joints.l_arm_mwx; joints.neck_ay];
            end
            joints_lower_limit = q_nom(joint_inds);
            joints_upper_limit = q_nom(joint_inds);
            leftArmConstraint = leftArmConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function leftLegConstraint = addLeftLegConstraint(robot)
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');
            leftLegConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.l_leg_kny;joints.r_leg_kny];
            joints_lower_limit = 30*pi/180*[1;1];
            joints_upper_limit = 120*pi/180*[1;1];
            leftLegConstraint = leftLegConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
    end
        
end