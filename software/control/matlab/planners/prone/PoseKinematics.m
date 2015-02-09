classdef PoseKinematics
    
    properties
        contacts
        options
        robot
        ik
        constraints
        shrink_factor = 0.2;
        not_collision_checked = [32];
        q_nom_original;
        contact_pts;
    end
    
    
    
    methods
        
        function obj = PoseKinematics(contacts,options)
            
            if isempty(contacts)
                disp('WARNING: NO CONTACTS SPECIFIED');
            end
            
            if nargin < 2
                options = struct();
            end
            
            % set the shrink factor
            if isfield(options,'shrink_factor')
                disp('shrink factor set to')
                obj.shrink_factor = options.shrink_factor;
                disp(options.shrink_factor)
            end
            disp('shrink factor set to')
            disp(obj.shrink_factor)
            
            obj.contacts = contacts;
            obj.options = options;
            
            
            if isfield(options,'robot')
                robot = options.robot;
            else
                
                % atlas convex hull
                atlas_urdf = [getenv('DRC_BASE'),'/software/models/atlas_v5/model_convex_hull.urdf'];
                options.floating = true;
                
                robot = RigidBodyManipulator(atlas_urdf,options);
                ground = RigidBodyFlatTerrain();
                robot = robot.setTerrain(RigidBodyFlatTerrain());
                robot = compile(robot);
                obj.robot = robot;
            end
            
            
            % Add the ground to robot
            ground = RigidBodyFlatTerrain();
            robot = robot.setTerrain(RigidBodyFlatTerrain());
            robot = compile(robot);
            obj.robot = robot;
            
            obj.contact_pts = PoseKinematics.initialize_contact_pts(obj.robot);
            
            %% Get q_seed, q_nom from options
            nom_data_string = [getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v5_fp.mat'];
            nom_data = load(nom_data_string);
            nq = obj.robot.getNumPositions;
            q_nom_original =  nom_data.xstar(1:nq);
            obj.q_nom_original = q_nom_original;
            
            
            %% Support manually exempting bodies from contact
            if isfield(options, 'remove_collision_bodies')
                rcb = options.remove_collision_bodies;
                for j=1:numel(rcb)
                    obj.not_collision_checked(end+1) = PoseKinematics.linkId(obj.robot,rcb{j});
                end
            end
            
            
            %% Add constraints
            obj.constraints = {};
            
            obj = obj.addSupportConstraints(contacts);
            obj = obj.addQuasiStaticConstraint(contacts);
            obj = obj.addCollisionConstraint();
            
            
        end
        
        function [q,F,info,infeasible_constraint] = solve(obj,options)
            
            if nargin < 2
                options = struct();
            end
            
            if isfield(options,'q_nom')
                q_nom = options.q_nom;
                
            elseif isfield(options,'nom')
                % options.nom should be a string, e.g. 'nom', 'prone', 'kneeling' . . .
                q_nom = obj.getStoredPose(options.nom);
                
            elseif isfield(obj.options,'q_nom')
                q_nom = obj.options.q_nom;
                
            elseif isfield(obj.options,'nom')
                % options.nom should be a string, e.g. 'nom', 'prone', 'kneeling' . . .
                q_nom = obj.getStoredPose(obj.options.nom);
                
            else
                q_nom = obj.q_nom_original;
            end
            
            
            
            if isfield(options, 'q_seed')
                q_seed = options.q_seed;
                
            elseif isfield(options, 'seed')
                q_seed = obj.getStoredPose(options.seed);
                
            elseif isfield(obj.options, 'q_seed')
                q_seed = obj.options.q_seed;
                
            elseif isfield(obj.options, 'seed')
                q_seed = obj.getStoredPose(obj.options.seed);
                
            else
                q_seed = obj.q_nom_original;
            end
            
            
            ik = InverseKinematics(obj.robot,q_nom,obj.constraints{:});
            ik = ik.setSolverOptions('snopt','iterationslimit',1e6);
            ik = ik.setSolverOptions('snopt','majoriterationslimit',200);
            ik = ik.setSolverOptions('snopt','print','snopt.out');
            [q,F,info,infeasible_constraint] = ik.solve(q_seed);
            disp(snoptInfo(info))
            info
            infeasible_constraint
            
            
        end
        
        % q_0 is the pose you don't want movement from
        function obj = addMovementConstraint(obj, q_0, bodies)
            % bodies should be a cell array, i.e. {'l_toe','r_toe'} . . .
            
            robot = obj.robot;
            
            m_constraints = cell(1,length(bodies));
            
            kinsol = robot.doKinematics(q_0);
            
            
            for j = 1:length(bodies)
                
                name = bodies{j};
                bds = robot.forwardKin(kinsol,obj.linkId(robot,name),obj.contact_pts(name));
                wp = WorldPositionConstraint(robot,obj.linkId(robot,name),...
                    obj.contact_pts(name),bds,bds);
                m_constraints{j} = wp;
                
            end
            
            % update the constraints
            obj.constraints = [obj.constraints, m_constraints];
            
            
        end
        
        
        % Adds the constraint that these bodies be on the ground
        function obj = addSupportConstraints(obj,contacts)
            
            for j = 1:length(contacts)
                name = contacts{j};
                id = obj.linkId(obj.robot,name);
                
                % constraint that it be touching the ground, WORLDPOSITIONCONSTRAINT
                bds = [nan;nan;0];
                pts = obj.contact_pts(name);
                s = size(pts);
                bds = repmat(bds,1,s(2)); % make sure this is the correct size
                % add it to constraints
                obj.constraints{end+1} = WorldPositionConstraint(obj.robot,id,pts,bds,bds);
                
                % remove it from collision bodies . . .
                obj.not_collision_checked(end+1) = id;
            end
        end
        
        
        % adds the quasiStatic constraint
        % add analogous get method for these
        function obj = addQuasiStaticConstraint(obj,contacts)
            qs = QuasiStaticConstraint(obj.robot);
            qs = qs.setShrinkFactor(obj.shrink_factor);
            
            for j = 1:length(contacts)
                name = contacts{j};
                id = obj.linkId(obj.robot,name);
                pts = obj.contact_pts(name);
                qs = qs.addContact(id,pts);
            end
            
            qs = qs.setActive(true);
            obj.constraints{end+1} = qs;
        end
        
        
        % add the collision constraints
        function obj = addCollisionConstraint(obj)
            
            nb = obj.robot.getNumBodies;
            active_collision_options.body_idx = setdiff(1:nb, obj.not_collision_checked);
            
            min_distance = 0.03;
            mdc = MinDistanceConstraint(obj.robot,min_distance,active_collision_options);
            
            aco_no_ground.body_idx = [2:nb];
            mdc_no_ground = MinDistanceConstraint(obj.robot,min_distance,aco_no_ground);
            
            obj.constraints{end+1} = mdc;
            obj.constraints{end+1} = mdc_no_ground;
        end
        
        function [mdc_eval, mdc_no_ground_eval] = checkCollisionConstraint(obj,q)
            kinsol = obj.robot.doKinematics(q);
            nb = obj.robot.getNumBodies;
            active_collision_options.body_idx = setdiff(1:nb, obj.not_collision_checked);
            
            min_distance = 0.01;
            mdc = MinDistanceConstraint(obj.robot,min_distance,active_collision_options);
            
            aco_no_ground.body_idx = [2:nb];
            mdc_no_ground = MinDistanceConstraint(obj.robot,min_distance,aco_no_ground);
            
            mdc_eval = mdc.eval(0,kinsol);
            mdc_no_ground_eval = mdc_no_ground.eval(0,kinsol);
        end
        
        
        function  [isSat, cstr_not_sat] = checkAllConstraints(obj,q)
            kinsol = obj.robot.doKinematics(q);
            isSat = 1;
            cstr_not_sat = {};
            for j=1:numel(obj.constraints)
                
                category = obj.constraints{j}.category;
                
                % QuasiStaticConstraint has different eval method . . 
                if category == -3
                    % note that it evaluates to 1 if it is satisfied, so we
                    % just subtract 1 to fix this.
                    val = obj.constraints{j}.checkConstraint(kinsol);
                    val = val -1;
                else
                    val = obj.constraints{j}.eval(0,kinsol);
                end
                
                if ~( abs(val) < 1e-5)
                    isSat = 0;
                    cstr_not_sat{end+1} = obj.constraints{j};
                end
            end
            
            
            if isSat==1
                disp('All constraints satisfied')
            else
                disp('Constraint violated')
            end
            
        end
        
        
        function drawTest(obj,q)
            v = obj.robot.constructVisualizer;
            v.draw(0,q);
        end
        
    end
    
    methods (Static)
        function c = initialize_contact_pts(robot)
            % This stores the master data for all the contact points.
            
            %% Copied over from test_poses
            l_face_cage = [0.313;0.154;0.622];
            r_face_cage = [0.313;-0.162;0.6228];
            face_cage_pts = [l_face_cage,r_face_cage];
            
            
            
            %% Initialize containers.Map
            
            c = containers.Map;
            
            c('l_toe') = robot.getBody(robot.findLinkId('l_foot')).getTerrainContactPoints('toe');
            c('l_heel') = robot.getBody(robot.findLinkId('l_foot')).getTerrainContactPoints('heel');
            c('r_toe') = robot.getBody(robot.findLinkId('r_foot')).getTerrainContactPoints('toe');
            c('r_heel') = robot.getBody(robot.findLinkId('r_foot')).getTerrainContactPoints('heel');
            
            c('l_knee') = [0.09485; 0.0; -0.0401];
            c('r_knee') = [0.09485; 0.0; -0.0401];
            c('r_hand') = [0.1, -0.1, 0, 0;
                -0.4, -0.4, -0.4, -0.4;
                0, 0, 0.1, -0.1];
            
            c('l_hand') = [0.1, -0.1, 0, 0;
                0.4, 0.4, 0.4, 0.4;
                0, 0, 0.1, -0.1];
            
            c('face_cage') = face_cage_pts;
            
            c('chest') = [0.258;0;0.1882];
            
            c('l_foot') = [c('l_toe'), c('l_heel')];
            c('r_foot') = [c('r_toe'), c('r_heel')];
            
            
            
            
        end
        
        % access method for the contact geometries
        function pts = getContactPts(name,robot)
            c = PoseKinematics.initialize_contact_pts(robot);
            pts = c(name);
        end
        
        function q = getStoredPose(name)
            
            poses = load('pose_data.mat');
            
            q = getfield(poses,name);
            
            
        end
        
        function id = linkId(robot,name)
            id =-1;
            
            if strcmp(name, 'l_toe')
                id =  robot.findLinkId('l_foot');
            elseif  strcmp(name,'r_toe')
                id =  robot.findLinkId('r_foot');
                
            elseif  strcmp(name,'r_knee')
                id =  robot.findLinkId('r_lleg');
                
            elseif strcmp(name,'l_knee')
                id =  robot.findLinkId('l_lleg');
                
            elseif  strcmp(name,'r_hand')
                id =  31;
                
            elseif strcmp(name,'l_hand')
                id =  18;
                
            elseif  strcmp(name,'face_cage')
                id =  robot.findLinkId('utorso');
                
            elseif  strcmp(name,'chest')
                id =  robot.findLinkId('utorso');
                
            elseif strcmp(name,'l_foot')
                id = robot.findLinkId('l_foot');
                
            elseif strcmp(name,'r_foot')
                id = robot.findLinkId('r_foot');
                
            elseif strcmp(name,'l_heel')
                id = robot.findLinkId('l_foot');
                
            elseif strcmp(name,'r_heel')
                id = robot.findLinkId('r_foot');
            end
            
            
            
        end
        
        
        % get the position in the world frame of a contact point given a
        % pose for the robot
        
        function pos = getPositionInWorldFrame(robot,q,name)
            kinsol = robot.doKinematics(q);
            id = PoseKinematics.linkId(robot,name);
            pts = PoseKinematics.getContactPts(name,robot);
            pos = robot.forwardKin(kinsol,id,pts);
        end
        
        
        function robot = addVisualContactPts(robot)
            
            c = PoseKinematics.initialize_contact_pts(robot);
            keySet = keys(c);
            
            for j=1:length(keySet)
                name = keySet{j};
                link = PoseKinematics.linkId(robot,name);
                pts = c(name);
                
                for k = 1:size(pts,2)
                    sphere =  RigidBodySphere(0.01,pts(:,k),[0;0;0]);
                    robot = robot.addVisualGeometryToBody(link,sphere);
                end
            end
            
            robot = compile(robot);
            
        end
        
        
        
        function drawCOM(robot,q)
            kinsol = robot.doKinematics(q);
            com_pos = robot.getCOM(kinsol);
            com_pos(3) = 0;
            
            lcmgl = LCMGLClient;
            lcmgl.glColor3f(1,0,0);
            lcmgl.sphere(com_pos,0.02,20,20);
            lcmgl.switchBuffers;
            v = robot.constructVisualizer;
            v.draw(0,q);
            
        end
        
        
        % just checks the quasiStaticConstraint
        function val = checkQuasiStaticConstraint(robot,q,shrink_factor,contacts)
            
            kinsol = robot.doKinematics(q);
            if nargin < 3
                shrink_factor = 0.6;
            end
            disp('shrink factor is')
            disp(shrink_factor);
            
            if nargin < 4
                contacts = IKTraj.getActiveContactsFromPose(robot,q);
            end
            
            disp('The contacts being used to check QuasiStatic are')
            disp(contacts)
            
            contact_pts = PoseKinematics.initialize_contact_pts(robot);
            qs = QuasiStaticConstraint(robot);
            qs = qs.setShrinkFactor(shrink_factor);
            
            for j = 1:length(contacts)
                name = contacts{j};
                id = PoseKinematics.linkId(robot,name);
                pts = contact_pts(name);
                qs = qs.addContact(id,pts);
            end
            
            if qs.checkConstraint(kinsol)
                disp('QuasiStaticConstraint Satisfied')
            else
                disp('NOT QUASISTATICALLY STABLE');
            end
        end
        
        
        
    end
    
    
    
    
end









































