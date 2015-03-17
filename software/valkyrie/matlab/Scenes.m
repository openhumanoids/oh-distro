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
        function robot = generate(scene, robot, world_link)
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

                    targetObjectPos = [0.7 0 1.0625];
                    targetObject = RigidBodyBox([.05 .05 .3], targetObjectPos, [0 0 0]);
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

                    targetObjectPos = [0.7 0 1.0625];
                    targetObject = RigidBodyBox([.05 .05 .3], targetObjectPos, [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
                case 'scene3'
                    table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, table);

                    targetObjectPos = [0.7 0 0.6];
                    targetObject = RigidBodyBox([.05 .05 .3], targetObjectPos, [0 0 0]);
                    robot = addGeometryToBody(robot, world_link, targetObject);
            end
        end
        
        function targetObjectPos = getTargetObjPos(scene)
            switch scene
                case 'scene1'
                    targetObjectPos = [0.7 0 1.0625];
                case 'scene2'
                    targetObjectPos = [0.7 0 1.0625];
                case 'scene3'
                    targetObjectPos = [0.7 0 0.6];
            end
        end
        
        function startPoseConstraints = setStartPoseConstraints(scene, robot)            
            quasiStaticConstraint = Scenes.addQuasiStaticConstraint(robot);
            [leftFootPosConstraint, leftFootQuatConstraint] = Scenes.addLeftFootConstraint(robot);
            [rightFootPosConstraint , rightFootQuatConstraint] = Scenes.addRightFootConstraint(robot);
            backConstraint = Scenes.addBackConstraint(robot);
            baseConstraint = Scenes.addBaseConstraint(robot);
            rightArmConstraint = Scenes.addRightArmConstraint(robot);
            leftLegConstraint = Scenes.addLeftLegConstraint(robot);
            switch scene
                case 'debris'
                    l_hand = robot.findLinkId('l_hand');
                    xyz_quat_start = [0.5969; -0.1587; 0.9; -0.2139; 0.6724; 0.3071; -0.6387];
                    l_hand_initial_position_constraint = WorldPositionConstraint(robot, l_hand, [0;0;0], xyz_quat_start(1:3), xyz_quat_start(1:3));
                    l_hand_initial_quat_constraint = WorldQuatConstraint(robot, l_hand, xyz_quat_start(4:7), 0*pi/180);
                    
                    startPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, backConstraint, baseConstraint, rightArmConstraint,leftLegConstraint,...
                                            l_hand_initial_position_constraint , l_hand_initial_quat_constraint};                            
                otherwise
                    startPoseConstraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,...
                                            rightFootQuatConstraint, backConstraint, baseConstraint, rightArmConstraint,leftLegConstraint};
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
        
        function [leftFootPosConstraint, leftFootQuatConstraint] = addLeftFootConstraint(robot)  
            l_foot = robot.findLinkId('l_foot');          
            point_in_link_frame = [0.0; 0.0; 0.0];
            ref_frame = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341;...
                         -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892;...
                         -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311;...
                         0.0, 0.0, 0.0, 1.0];
            lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            ref_frame = inv(ref_frame);
            leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);
            
            leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, [0.99999680768841015; -0.0024891132733300568; 0.00043417407699420605; -2.0517608182535892e-05], 0.0, [0.0, 1.0]);
        end
        
        function [rightFootPosConstraint, rightFootQuatConstraint] = addRightFootConstraint(robot)  
            r_foot = robot.findLinkId('r_foot');
            point_in_link_frame = [0.0; 0.0; 0.0];
            ref_frame = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833;...
                         4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941;...
                         -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948;...
                         0.0, 0.0, 0.0, 1.0];
            lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
            ref_frame = inv(ref_frame);
            rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


            rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, [0.99999684531339206; 0.0024841562616134435; 0.00037137837375452614; 2.0224619435999976e-05], 0.0, [0.0, 1.0]);
        end
        
        function backConstraint = addBackConstraint(robot)
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');
            backConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
            joints_lower_limit = [-0.08726646259971647; -0.08726646259971647; -inf];
            joints_upper_limit = [0.08726646259971647; 0.08726646259971647; inf];
            backConstraint = backConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function baseConstraint = addBaseConstraint(robot)
            S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
            q_nom = S.xstar(1:robot.getNumPositions());
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');
            baseConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
            joints_lower_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
            joints_upper_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
            baseConstraint = baseConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
        end
        
        function rightArmConstraint = addRightArmConstraint(robot)
            S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
            q_nom = S.xstar(1:robot.getNumPositions());   
            joints = Point(robot.getStateFrame, (1:robot.getStateFrame.dim)');     
            rightArmConstraint = PostureConstraint(robot, [-inf, inf]);
            joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
            joints_lower_limit = q_nom(joint_inds);
            joints_upper_limit = q_nom(joint_inds);
            rightArmConstraint = rightArmConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
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