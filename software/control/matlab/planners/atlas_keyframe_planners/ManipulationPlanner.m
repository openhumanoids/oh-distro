classdef ManipulationPlanner < KeyframePlanner
    % USAGE
    % ManipulationPlanner manipulationPlanner(r);
    % manipulationPlanner.generateAndPublishPlan(vargin);
    % cache = manipulationPlanner.getPlanCache();
    %
    properties
        plan_pub
        map_pub        
        restrict_feet
        planning_mode % 1 if ik sequence is on, 2 if use IK only, 3 if use teleop
    end
    
    methods
        function obj = ManipulationPlanner(r,hardware_mode)
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            
            obj.plan_cache.num_breaks = 4;
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.map_pub = AffIndexedRobotPlanPublisher('CANDIDATE_MANIP_MAP',true,joint_names);
            obj.restrict_feet=true;
            obj.planning_mode = 1;
        end
        
        function setPlanningMode(obj,val)
            obj.planning_mode  = val;
        end
        
        function generateAndPublishManipulationPlan(obj,varargin)
            
            switch nargin
                case 7
                    x0 = varargin{1};
                    ee_names= varargin{2};
                    ee_loci = varargin{3};
                    timeIndices = varargin{4};
                    postureconstraint = varargin{5};
                    goal_type_flags = varargin{6};
                    % runs IK sequence but its  slow.
                    % Given N constraitns, iksequence needs atleast N break points
                    % which is slow.
                    %runOptimization(obj,x0,ee_names,ee_loci,timeIndices);
                    
                    % Point wise IK, much faster, linear complexity.
                    is_manip_map =false;
                    fprintf('EELoci function being called');
                    runOptimizationForManipMotionMapOrPlanGivenEELoci(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint,is_manip_map,goal_type_flags);
                otherwise
                    error('Incorrect usage of generateAndPublishManipulationPlan in Mnaip Planner. Undefined number of vargin.')
            end
            
        end
        
        function generateAndPublishManipulationMap(obj,x0,ee_names,ee_loci,affIndices)
            is_manip_map =true;
            runOptimizationForManipMotionMapOrPlanGivenEELoci(obj,x0,ee_names,ee_loci,affIndices,[],is_manip_map);
        end
        
        function runOptimizationForManipMotionMapOrPlanGivenEELoci(obj,x0,ee_names,ee_loci,Indices,postureconstraint,is_manip_map,goal_type_flags)
            if(is_manip_map)
                disp('Generating manip map...');
                send_status(3,0,0,'Generating manip map...');
            else
                disp('Generating manip plan...');
                send_status(3,0,0,'Generating manip plan...');
            end
            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
            
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],2);
            
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            
            
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            head_pose0(1:3)=nan(3,1); % only set head orientation not position
            head_pose0_relaxed.min=head_pose0-[1e-2*ones(3,1);1e-2*ones(4,1)];
            head_pose0_relaxed.max=head_pose0+[1e-2*ones(3,1);1e-2*ones(4,1)];
            
            
            
            % compute fixed COM goal
            gc = contactPositions(obj.r,q0);
            k = convhull(gc(1:2,:)');
            com0 = getCOM(obj.r,q0);
            %   comgoal = [mean(gc(1:2,k(1:end-1)),2);com0(3)];
            %   comgoal = com0; % DOnt move com for now as this is pinned manipulation
            comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            
            
            % compute EE trajectories
            r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
            
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);megaclear
            utorso_pose0_relaxed = utorso_pose0;
            utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-2*ones(4,1)];
            utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-2*ones(4,1)];
            % utorso_pose0 = utorso_pose0(1:3);

            T_palm_grasp = HT([0.05;0;0],0,0,0); % We evaluate the achievement of hand grasps based upon a notional grasp point
            T_grasp_palm = inv_HT(T_palm_grasp);
            
            ind = getActuatedJoints(obj.r);
            cost = getCostVector(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.MajorIterationsLimit = 100;
            ikoptions.shrinkFactor = 0.8;
            ikoptions.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
            ikoptions.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
            
            % Might be worth preventing leaning forward too much also
            
            if(~is_manip_map)
                obj.plan_cache.ks = ActionSequence(); % clear plan cache
                %obj.rfootT = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],1);
                %obj.lfootT = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],1);
                %obj.rhandT = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
                %obj.lhandT = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);
                % Dont move head during keyframe modification?
                % headT  = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],1);
                % kc_headT = ActionKinematicConstraint(obj.r, obj.head_body,[0;0;0],headT,[0,1],'headT');
                % obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_headT);
                obj.cacheLHandPose([0 0],'lhand0',l_hand_pose0);
                obj.cacheRHandPose([0 0],'rhand0',r_hand_pose0);
                obj.cacheLFootPoseAsContactConstraint([0 0],'lfoot0',l_foot_pose0);
                obj.cacheRFootPoseAsContactConstraint([0 0],'rfoot0',r_foot_pose0);
            end
            
            % Solve IK for each element i n EE LOCII
            timeIndices=[];
            if(is_manip_map)
                timeIndices = unique([Indices.time]);
            else
                timeIndices = unique(Indices);
            end
            N = length(timeIndices);
            plan_Indices=[];
            q_guess =q0;
            %plan_
            lhand_const_plan = [];
            rhand_const_plan = [];
            rhand_const_plan = [];
            rhand_const_plan = [];
            for i=1:length(timeIndices),
                tic;
                
                %l_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                if(goal_type_flags.lh ~=2)
                    lhand_const.min = l_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                    lhand_const.max = l_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                else
                    lhand_const.type = 'gaze';
                    lhand_const.gaze_target = ee_loci(1:3,i);
                    lhand_const.gaze_axis = [1;0;0];%obj.lhandT = l_ee_goal;
                    lhand_const.gaze_conethreshold = pi/18;
                end
                %r_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                if(goal_type_flags.rh ~=2)
                    rhand_const.min = r_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                    rhand_const.max = r_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                else
                    rhand_const.type = 'gaze';
                    rhand_const.gaze_target = ee_loci(1:3,i);
                    rhand_const.gaze_axis = [1;0;0];
                    rhand_const.gaze_conethreshold = pi/18;
                end
                if(goal_type_flags.h ==2)
                    head_const.type = 'gaze';
                    head_const.gaze_target = ee_loci(1:3,i);
                    head_const.gaze_axis = [1;0;0];
                    head_const.gaze_conethreshold = pi/12;
                else%obj.lhandT = l_ee_goal;
                    head_const = [];
                end
                %l_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
                %                 lfoot_const.min = l_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                %                 lfoot_const.max = l_foot_pose0+1e-2*[ones(3,1);ones(4,1)];
                %r_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
                %                 rfoot_const.min = r_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                %                 rfoot_const.max = r_foot_pose0+1e-2*[onr_hand_pose0es(3,1);ones(4,1)];
                r_foot_pose = r_foot_pose0;
                T_world_r_foot = [quat2rotmat(r_foot_pose(4:7)) r_foot_pose(1:3);0 0 0 1];
                r_foot_pts_pose = T_world_r_foot*[r_foot_pts;ones(1,num_r_foot_pts)];
                r_foot_pts_pose = [r_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_r_foot_pts),r_foot_pose(4:7))];
                rfoot_const.min = r_foot_pts_pose-1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                rfoot_const.max = r_foot_pts_pose+1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                rfoot_const_static_contact = rfoot_const;
                rfoot_const_static_contact.contact_state = ...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
                
                l_foot_pose = l_foot_pose0;k
                T_world_l_foot = [quat2rotmat(l_foot_pose(4:7)) l_foot_pose(1:3);0 0 0 1];
                l_foot_pts_pose = T_world_l_foot*[l_foot_pts;ones(1,num_l_foot_pts)];
                l_foot_pts_pose = [l_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_l_foot_pts),l_foot_pose(4:7))];
                lfoot_const.min = l_foot_pts_pose-1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                lfoot_const.max = l_foot_pts_pose+1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                lfoot_const_static_contact = lfoot_const;
                lfoot_const_static_contact.contact_state = ...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
                ind = [];
                if(is_manip_map)
                    plan_Indices(i).time=Indices(i).time;
                    plan_Indices(i).aff_type=Indices(i).aff_type;
                    plan_Indices(i).aff_uid=Indices(i).aff_uid;
                    plan_Indices(i).num_ees=0;
                    plan_Indices(i).ee_name=[];
                    plan_Indices(i).dof_name=[];
                    plan_Indices(i).dof_value=[];
                    plan_Indices(i).dof_pose=[];
                    plan_Indices(i).dof_reached = [];
                    ind=find([Indices.time]==timeIndices(i));
                else
                    ind=find(Indices==timeIndices(i));
                end
                
                % Find all active constraints at current index
                for k=1:length(ind),
                    
                    if(strcmp('left_palm',ee_names{ind(k)}))
                        l_ee_goal = ee_loci(:,ind(k));
                        lhandT = zeros(6,1);
                        T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                        T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l_sandia;
                        lhandT(1:3) = T_world_hand_l(1:3,4);
                        lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                        l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                        T_world_grasp_l = T_world_palm_l * T_palm_grasp;
                        lgraspT = zeros(6,1);
                        lgraspT(1:3) = T_world_grasp_l(1:3,4);
                        lgraspT(4:6) =rotmat2rpy(T_world_grasp_l(1:3,1:3));
                        l_grasp_pose = [lgraspT(1:3); rpy2quat(lgraspT(4:6))];obj.cacheRHandPose([0 0],'rhand0',r_hand_pose0);
                        lhand_const.min = l_hand_pose-0*1e-4*[ones(3,1);ones(4,1)];
                        lhand_const.max = l_hand_pose+0*1e-4*[ones(3,1);ones(4,1)];
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('l_hand')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_namr_ee_goale;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indices(i).dof_value;Indices(ind(k)).dof_value(1)];
                            %plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose l_hand_pose];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose l_grasp_pose];
                        else
                            if(i==length(timeIndices))
                                %obj.lhandT = l_ee_goal;                                
                                obj.cacheLHandPose([1 1],'lhandT',l_hand_pose);
                            else
                                N= length(ind);
                                obj.cacheLHandPose([i/N i/N],['lhand' num2str(i-1)],l_hand_pose);
                            end
                        end
                    end
                    
                    if(strcmp('right_palm',ee_names{ind(k)}))
                        r_ee_goal = ee_loci(:,ind(k));
                        rhandT = zeros(6,1);
                        T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                        T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r_sandia;
                        rhandT(1:3) = T_world_hand_r(1:3,4);
                        rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                        r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                        T_world_grasp_r = T_world_palm_r * T_palm_grasp;
                        rgraspT = zeros(6,1);
                        rgraspT(1:3) = T_world_grasp_r(1:3,4);
                        rgraspT(4:6) =rotmat2rpy(T_world_grasp_r(1:3,1:3));
                        r_grasp_pose = [rgraspT(1:3); rpy2quat(rgraspT(4:6))];
                        if(goal_type_flags.rh ~= 2)
                            rhand_const.min = r_hand_pose-0*1e-4*[ones(3,1);ones(4,1)];
                            rhand_const.max = r_hand_pose+0*1e-4*[ones(3,1);ones(4,1)];
                        end
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('r_hand')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_name;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indiceks(i).dof_value;Indices(ind(k)).dof_value(1)];
                            %plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose r_hand_pose];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose r_grasp_pose];
                        else
                            if(i==length(timeIndices))
                                %obj.rhandT = r_ee_goal;                               
                               obj.cacheRHandPose([1 1],'rhandT',r_hand_pose);
                            else
                                N= length(ind);
                                obj.cacheRHandPose([i/N i/N],['rhand' num2str(i-1)],r_hand_pose);
                            end
                        end
                    end
                    
                    if(strcmp('l_foot',ee_names{ind(k)}))
                        lfootT = ee_loci(:,ind(k));
                        l_foot_pose = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                        T_world_l_foot = [quat2rotmat(l_foot_pose(4:7)) l_foot_pose(1:3);0 0 0 1];
                        l_foot_pts_pose = T_world_l_foot*[l_foot_pts;ones(1,num_l_foot_pts)];
                        l_foot_pts_pose = [l_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_l_foot_pts),l_foot_pose(4:7))];
                        lfoot_const.min = l_foot_pts_pose-1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                        lfoot_const.max = l_foot_pts_pose+1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                        lfoot_const_static_contact = lfoot_const;
                        lfoot_const_static_contact.contact_state = ...
                            {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('l_foot')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_name;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indices(i).dof_value;Indices(ind(k)).dof_value(1)];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose l_foot_pose];
                        else
                            if(i==length(timeIndices))
                                % obj.lfootT = l_foot_pose;                                
                                obj.cacheLFootPoseAsContactConstraint([1 1],'lfootT',l_foot_pose);
                            else
                                N= length(ind);
                                obj.cacheLFootPoseAsContactConstraint([i/N i/N],['lfoot' num2str(i-1)],l_foot_pose);
                            end
                        end
                    end
                    
                    if(strcmp('r_foot',ee_names{ind(k)}))
                        rfootT = ee_loci(:,ind(k));
                        r_foot_pose = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                        T_world_r_foot = [quat2rotmat(r_foot_pose(4:7)) r_foot_pose(1:3);0 0 0 1];
                        r_foot_pts_pose = T_world_r_foot*[r_foot_pts;ones(1,num_r_foot_pts)];
                        r_foot_pts_pose = [r_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_r_foot_pts),r_foot_pose(4:7))];
                        rfoot_const.min = r_foot_pts_pose-1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                        rfoot_const.max = r_foot_pts_pose+1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                        rfoot_const_static_contact = rfoot_const;
                        rfoot_const_static_contact.contact_state = ...
                            {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('r_foot')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_name;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indices(i).dof_value;Indices(ind(k)).dof_value(1)];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose r_foot_pose];
                        else
                            if(i==length(timeIndices))
                                % obj.rfootT = r_foot_pose;                               
                                obj.cacheRFootPoseAsContactConstraint([1 1],'rfootT',r_foot_pose);
                            else
                                N= length(ind);
                                obj.cacheRFootPoseAsContactConstraint([i/N i/N],['rfoot' num2str(i-1)],r_foot_pose);
                            end
                        end
                    end
                end
           
                ikoptions.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
                ikoptions.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
                ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
                ikoptions.q_nom = q_guess;
                if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                    if(is_manip_map)
                        % dont use r_foot_pts here (this is for driving), no quasi static flag
                        ikoptions.quasiStaticFlag = false;
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_pose0,...
                            obj.head_body,[0;0;0],head_pose0_relaxed,...
                            obj.r_foot_body,[0;0;0],rfoot_const, ...
                            obj.l_foot_body,[0;0;0],lfoot_const, ...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    else
                        ikoptions.quasiStaticFlag = true;
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_pose0,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    end
                else
                    if(is_manip_map)
                        % dont use r_foot_pts here (this is for driving), no quasi static flag
                        ikoptions.quasiStaticFlag = false;
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_pose0,...
                            obj.head_body,[0;0;0],head_pose0_relaxed,...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    else
                        ikoptions.quasiStaticFlag = false;
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_pose0,...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    end
                    
                end % end if(~obj.isBDIManipMode())
                
                snopt_info_vector(i) = snopt_info;
                q_guess =q(:,i);
                toc;
                if(snopt_info > 10)
                    warning(['The IK fails at ',num2str(i)]);
		    send_status(4,0,0,sprintf('snopt_info = %d. Manip plan IK is not good at %d.',snopt_info,i));
                end

                %q_d(:,i) = q(ind,i);
            end
            
            % publish robot map
            if(is_manip_map)
                disp('Publishing manip map...');
            else
                disp('Publishing manip plan...');
            end
            
            utime = now() * 24 * 60 * 60;
            if(is_manip_map)
                
                % Keep the largest consecutive portion of the plan for each
                % dof such that the end-effector is close to desired
                ee_temp = [];
                for i=1:numel(plan_Indices)
                    kinsol_tmp = doKinematics(obj.r,q(:,i));
                    for j=1:numel(plan_Indices(i).ee_name)
                        ee_name = plan_Indices(i).ee_name(j);
                        map_pose = [];
                        if (strcmp('l_hand',ee_name))
                            map_pose = forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                        elseif (strcmp('r_hand',ee_name))
                            map_pose = forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                        elseif (strcmp('l_foot',ee_name))
                            map_pose = forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                        elseif (strcmp('r_foot',ee_name))
                            map_pose = forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                        end;
                        rpy = quat2rpy(map_pose(4:7));
                        T_world_m_hand = HT(map_pose(1:3),rpy(1), rpy(2), rpy(3));
                        %map_pose = T_palm_grasp * obj.T_hand_palm_l_sandia * map_pose;
                        T_world_m_grasp = T_world_m_hand * obj.T_hand_palm_l_sandia * T_palm_grasp;
                        map_pose = [T_world_m_grasp(1:3,4); rpy2quat(rotmat2rpy(T_world_m_hand(1:3,1:3)))];
                        desired_pose = plan_Indices(i).dof_pose(:,j);
                        pos_dist = sqrt(sum((desired_pose(1:3)-map_pose(1:3)).^2));
                        
                        if (numel(ee_temp) < j)
                            ee_temp(j).map_pose = [];
                            ee_temp(j).dof_reached = [];
                            ee_temp(j).pos_dist = [];
                            ee_temp(j).ee_name = ee_name;
                        end;
                        
                        if (strcmp('l_foot', ee_name) || strcmp('r_foot', ee_name))
                            pos_dist_threshold = 100;
                        else
                            pos_dist_threshold = 0.025;
                        end;
                        ee_temp(j).map_pose = [ee_temp(j).map_pose map_pose];
                        ee_temp(j).dof_reached = [ee_temp(j).dof_reached (pos_dist < pos_dist_threshold)];
                        ee_temp(j).pos_dist = [ee_temp(j).pos_dist pos_dist];
                        
                        % Is the resulting pose sufficiently close to the
                        % desired pose?
                        plan_Indices(i).dof_reached = [plan_Indices(i).dof_reached (pos_dist < pos_dist_threshold)];
                    end;
                end;
                
                min_dof_idx = 1;
                max_dof_idx = length(ee_temp(1).dof_reached);
                for i=1:numel(ee_temp)
                    if(~isempty(find(~ee_temp(i).dof_reached)))
                        if (strcmp('l_hand',ee_temp(i).ee_name))
                            [~,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(l_hand_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        elseif (strcmp('r_hand',ee_temp(i).ee_name))
                            [~,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(r_hand_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        elseif (strcmp('l_foot',ee_temp(i).ee_name))
                            [~,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(l_foot_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        elseif (strcmp('r_foot',ee_temp(i).ee_name))
                            [~,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(r_foot_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        end
                        
                        fprintf (1, 'Current index is %d whose distance is %.4f\n', jcur, ee_temp(i).pos_dist(jcur));
                        % find the min and max indices of the dof value
                        jmax = jcur;
                        jmin = jcur;
                        while (jmax < size(ee_temp(i).dof_reached,2) && ee_temp(i).dof_reached(jmax+1) == 1)
                            jmax = jmax + 1;
                        end;
                        while (jmin > 1 && ee_temp(i).dof_reached(jmin) == 1)
                            jmin = jmin - 1;
                        end;
                        
                        ee_temp(i).min_dof_reachable_idx = jmin;
                        ee_temp(i).max_dof_reachable_idx = jmax;
                        
                        if (jmin > min_dof_idx)
                            min_dof_idx = jmin;
                        end;
                        
                        if (jmax < max_dof_idx)
                            max_dof_idx = jmax;
                        end;
                    end;
                end;
                
                fprintf (1, 'Filtering manip map for reachability: Keeping %d of %d original plan\n', max_dof_idx-min_dof_idx+1, length(timeIndices));
                stat = sprintf('Filtering manip map for reachability: Keeping %d of %d original plan\n', max_dof_idx-min_dof_idx+1, length(timeIndices));
                send_status(3,0,0,stat);
                
                stat = sprintf('Filtering manip map for reachability: Keeping %d of %d original plan\n', max_dof_idx-min_dof_idx+1, length(timeIndices));
                send_status(3,0,0,stat);
                
                timeIndices = timeIndices(min_dof_idx:max_dof_idx);
                
                plan_Indices = plan_Indices(min_dof_idx:max_dof_idx);
                q = q(:,min_dof_idx:max_dof_idx);
                
                xtraj = zeros(getNumStates(obj.r),length(timeIndices));
                xtraj(1:getNumDOF(obj.r),:) = q;
                
                % Keep the largest consequtive portion such that the
                % end-effector is sufficiently close to desired
                
                obj.map_pub.publish(xtraj,plan_Indices,utime);
            else
                xtraj = zeros(getNumStates(obj.r)+2,length(timeIndices));
                xtraj(1,:) = 0*timeIndices;
                keyframe_inds = unique(round(linspace(1,length(timeIndices),obj.plan_cache.num_breaks))); % no more than ${obj.num_breaks} keyframes
                xtraj(1,keyframe_inds) = 1.0;
                xtraj(2,:) = 0*timeIndices;
                xtraj(3:getNumDOF(obj.r)+2,:) = q;
                
                s = (timeIndices-min(timeIndices))/(max(timeIndices)-min(timeIndices));
                
                %timeIndices
                
                fprintf('Max : %f - Min : %f', max(timeIndices), min(timeIndices));
                
                
                
                s_sorted = sort(s);
                s_breaks = s_sorted(keyframe_inds);
                
                % update plan cache
                obj.plan_cache.s_breaks = s_breaks;
                obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
                obj.cachePelvisPose([0 1],'pelvis',pelvis_pose0);
                obj.plan_cache.quasiStaticFlag = false;
                
                % calculate end effectors breaks via FK.
                for brk =1:length(s_breaks),
                    q_break = obj.plan_cache.qtraj.eval(s_breaks(brk));
                    kinsol_tmp = doKinematics(obj.r,q_break);
                    rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                    lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                    rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                    lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                    head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
                    ruarm_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_uarm_body,[0;0;0],2);
                    luarm_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_uarm_body,[0;0;0],2); 
                end
                s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
                s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
                s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
                s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
                s_total_lel =  sum(sqrt(sum(diff(luarm_breaks(1:3,:),1,2).^2,1)));
                s_total_rel =  sum(sqrt(sum(diff(ruarm_breaks(1:3,:),1,2).^2,1)));
                s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
                s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),max(s_total_head,max(s_total_lel,s_total_rel)));
                s_total = max(s_total,0.01);
                
                
                
                dqtraj=fnder(obj.plan_cache.qtraj,1); 
                sfine = linspace(s(1),s(end),50);
                Tmax_joints = max(max(abs(eval(dqtraj,sfine)),[],2))/obj.plan_cache.qdot_desired;
                Tmax_ee  = (s_total/obj.plan_cache.v_desired);
                ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
                obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
                brkpts =logical(zeros(1,length(timeIndices))==1);
                if(~isempty(postureconstraint))
                    timetags = [postureconstraint.utime];
                    if(length(unique(timetags)) > 1)
                        for k=1:length(timetags),
                            brkpts = brkpts|(timeIndices == timetags(k));
                        end
                    end
                    brkpts_shiftright=circshift(brkpts,[ 0 -1]);
                    xtraj(2,:) = brkpts|brkpts_shiftright;
                    
                    unique_transitions = unique(timetags);%get all values for a given time index
                    cnt = 1;
                    for j=1:length(unique_transitions),
                        s_transition =(unique_transitions(j)-min(timeIndices))/(max(timeIndices)-min(timeIndices));
                        ind = find(unique_transitions(j)==timetags);
                        num_joints = length(ind);
                        num_l_joints = 0;
                        num_r_joints = 0;
                        ind_l =[];ind_r =[];
                        % figure out if it is bihanded or single hand grasp transition.
                        for(k=1:num_joints),
                            if(~isempty(regexp(char(postureconstraint(ind(k)).joint_name),'left_')))
                                num_l_joints = num_l_joints+1;
                                ind_l = [ind_l ind(k)];
                            else
                                num_r_joints = num_r_joints+1;
                                ind_r =[ind_r ind(k)];
                            end
                        end
                        
                        if(num_l_joints>0)
                            G(cnt).utime =  s_transition.*(s_total/obj.plan_cache.v_desired);
                            G(cnt).num_joints=round(num_l_joints);
                            G(cnt).joint_name=javaArray('java.lang.String', num_l_joints);
                            G(cnt).joint_position=zeros(1,num_l_joints);
                            for k=1:num_l_joints,
                                G(cnt).joint_name(k) = postureconstraint(ind_l(k)).joint_name;
                                G(cnt).joint_position(k) = postureconstraint(ind_l(k)).joint_position;
                            end
                            G(cnt).affordance_uid = 0;
                            G(cnt).grasp_on = false;
                            G(cnt).grasp_type = 0; % LEFT
                            G(cnt).power_grasp=false;
                            pose = drc.position_3d_t();
                            pose.translation = drc.vector_3d_t();
                            pose.rotation = drc.quaternion_t();
                            pose.rotation.w =1.0;
                            G(cnt).hand_pose = pose;
                            cnt = cnt+1;
                        end
                        
                        if(num_r_joints>0)
                            G(cnt).utime =  s_transition.*(s_total/obj.plan_cache.v_desired);
                            G(cnt).num_joints=round(num_r_joints);
                            G(cnt).joint_name=javaArray('java.lang.String', num_r_joints);
                            G(cnt).joint_position=zeros(1,num_r_joints);
                            for k=1:num_r_joints,
                                G(cnt).joint_name(k) = postureconstraint(ind_r(k)).joint_name;
                                G(cnt).joint_position(k) = postureconstraint(ind_r(k)).joint_position;
                            end
                            G(cnt).affordance_uid = 0;
                            G(cnt).grasp_on = false;
                            G(cnt).grasp_type = 1; % RIGHT
                            G(cnt).power_grasp=false;
                            pose = drc.position_3d_t();
                            pose.translation = drc.vector_3d_t();
                            pose.rotation = drc.quaternion_t();
                            pose.rotation.w =1.0;
                            G(cnt).hand_pose = pose;
                            cnt = cnt+1;
                        end
                    end
                    %utime = 0;
                    obj.plan_pub.publish(xtraj,ts,utime,snopt_info_vector,G);
                else
                    obj.plan_pub.publish(xtraj,ts,utime,snopt_info_vector);
                end
                
            end
        end
        
        function cost = getCostVector(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 100;
            cost.base_y = 100;
            cost.base_z = 100;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 100;
            cost.back_bkz = 1e4;
            cost.back_bky = 1e4;
            cost.back_bkx = 1e4;
            cost.neck_ay =  100;
            cost.l_arm_usy = 1;
            cost.l_arm_shx = 1;
            cost.l_arm_ely = 1;
            cost.l_arm_elx = 1;
            cost.l_arm_uwy = 1;
            cost.l_arm_mwx = 1;
            if(~obj.isBDIManipMode())
                val = 1;
            else
                val = 100; % high cost on moving legs
            end
            cost.l_leg_hpz = val;
            cost.l_leg_hpx = val;
            cost.l_leg_hpy = val;
            cost.l_leg_kny = val;
            cost.l_leg_aky = val;
            cost.l_leg_akx = val;
            cost.r_arm_usy = cost.l_arm_usy;
            cost.r_arm_shx = cost.l_arm_shx;
            cost.r_arm_ely = cost.l_arm_ely;
            cost.r_arm_elx = cost.l_arm_elx;
            cost.r_arm_uwy = cost.l_arm_uwy;
            cost.r_arm_mwx = cost.l_arm_mwx;
            cost.r_leg_hpz = cost.l_leg_hpz;
            cost.r_leg_hpx = cost.l_leg_hpx;
            cost.r_leg_hpy = cost.l_leg_hpy;
            cost.r_leg_kny = cost.l_leg_kny;
            cost.r_leg_aky = cost.l_leg_aky;
            cost.r_leg_akx = cost.l_leg_akx;
            cost = double(cost);
        end
        
    end% end methods
end% end classdef
