classdef ReachingPlanner < KeyframePlanner
    % USAGE
    % ReachingPlanner reachingPlanner(r);
    % reachingPlanner.generateAndPublishPlan(vargin);
    % cache = reachingPlanner.getPlanCache();
    %
    properties
        plan_pub
        restrict_feet
        planning_mode % 1 if ik sequence is on, 2 if use IK only, 3 if use teleop
    end
    
    methods
        function obj = ReachingPlanner(r,hardware_mode)
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.plan_cache.num_breaks = 4;
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.restrict_feet=true;
            obj.planning_mode = 1;
        end
       
        function setPlanningMode(obj,val)
            obj.planning_mode  = val;
        end
        
        function generateAndPublishReachingPlan(obj,varargin)
            
            switch nargin
                case 8
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags =varargin{7};
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,goal_type_flags);
                case 9
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags =varargin{7};
                    q_desired = varargin{8};
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,goal_type_flags,q_desired);
                otherwise
                    error('Incorrect usage of generateAndPublishReachingPlan in Reaching Planner. Undefined number of inputs.')
            end
        end
        
        function runOptimization(obj,varargin)
            
            q_desired= [];
            rh_ee_goal= [];
            lh_ee_goal= [];
            rf_ee_goal= [];
            lf_ee_goal= [];
            h_ee_goal = [];
            goal_type_flags.lh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
            goal_type_flags.rh = 0;
            goal_type_flags.h  = 0;
            goal_type_flags.lf = 0;
            goal_type_flags.rf = 0;
            switch nargin
                case 8
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags= varargin{7};
                case 10
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags= varargin{7};
                    q_desired = varargin{8};
                otherwise
                    error('Incorrect usage of runOptimization in Manip Planner. Undefined number of vargin.')
            end
            
            
            disp('Generating plan...');
            send_status(3,0,0,'Generating  plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0);
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,2);
            
            % compute fixed COM goal
            gc = contactPositions(obj.r,q0);
            k = convhull(gc(1:2,:)');
            com0 = getCOM(obj.r,q0);
            %   comgoal = [mean(gc(1:2,k),2);com0(3)];
            %   comgoal = com0; % DOnt move com for now as this is pinned manipulation
            
            
            % get hand positions
            
            % compute EE trajectories
            r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);

            
            % Get head position
            
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            %======================================================================================================
            
            obj.restrict_feet=true;
            
            if(isempty(rh_ee_goal))
                rh_ee_goal = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
                rhandT  = rh_ee_goal(1:6);
                rhandT = [nan;nan;nan;nan;nan;nan];
            else
                if(goal_type_flags.rh ~=2)
                    rhandT = zeros(6,1);
                    % Desired position of palm in world frame
                    T_world_palm_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
                    T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r_sandia;
                    rhandT(1:3) = T_world_hand_r(1:3,4);
                    rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                else
                    rhandT = rh_ee_goal(1:6);
                end
            end
            
            if(isempty(lh_ee_goal))
                lh_ee_goal = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);
                lhandT  = lh_ee_goal(1:6);
                lhandT = [nan;nan;nan;nan;nan;nan];
            else
                if(goal_type_flags.lh ~= 2)
                    lhandT = zeros(6,1);
                    % Desired position of palm in world frame
                    T_world_palm_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
                    T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l_sandia;
                    lhandT(1:3) = T_world_hand_l(1:3,4);
                    lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                else
                    lhandT = lh_ee_goal(1:6);
                end
            end
            
            if(isempty(rf_ee_goal))
                rf_ee_goal = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,1);
                rfootT  = rf_ee_goal(1:6,:);
                % rfootT = [nan;nan;nan;nan;nan;nan];
            else
                obj.restrict_feet=false;
                rfootT = zeros(6,num_r_foot_pts);
                % Desired position of palm in world frame
                for k = 1:num_r_foot_pts
                    T_world_foot_r = HT(rf_ee_goal(1:3,k),rf_ee_goal(4,k),rf_ee_goal(5,k),rf_ee_goal(6,k));
                    rfootT(1:3,k) = T_world_foot_r(1:3,4);
                    rfootT(4:6,k) =rotmat2rpy(T_world_foot_r(1:3,1:3));
                end
            end
            
            if(isempty(lf_ee_goal))
                lf_ee_goal = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,1);
                lfootT  = lf_ee_goal(1:6,:);
                %lfootT = [nan;nan;nan;nan;nan;nan];
            else
                obj.restrict_feet=false;
                lfootT = zeros(6,num_l_foot_pts);
                % Desired position of palm in world frame
                for k = 1:num_l_foot_pts
                    T_world_foot_l = HT(lf_ee_goal(1:3,k),lf_ee_goal(4,k),lf_ee_goal(5,k),lf_ee_goal(6,k));
                    lfootT(1:3,k) = T_world_foot_l(1:3,4);
                    lfootT(4:6,k) =rotmat2rpy(T_world_foot_l(1:3,1:3));
                end
            end
    
            if(isempty(h_ee_goal))
                h_ee_goal = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],1);
                headT  = h_ee_goal(1:6);
            else
                headT = zeros(6,1);
                % Desired position of head in world frame
                T_world_head = HT(h_ee_goal(1:3),h_ee_goal(4),h_ee_goal(5),h_ee_goal(6));
                headT(1:3) = T_world_head(1:3, 4);
                headT(4:6) =rotmat2rpy(T_world_head(1:3,1:3));
            end
            
            %===========================================================================
            % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
            head_gaze_target = [];
            lhand_gaze_target = [];
            rhand_gaze_target = [];
            if(goal_type_flags.h == 2)
                % headT(1:3) is actually object pos
                head_gaze_target = headT(1:3);
            end
            if(goal_type_flags.lh == 2)
                % lhandT(1:3) is actually object pos
                lhand_gaze_target = lhandT(1:3);
            end
            if(goal_type_flags.rh == 2)
                % rhandT(1:3) is actually object pos
                rhand_gaze_target = rhandT(1:3);
            end
            %===========================================================================
            
            if(goal_type_flags.rh ~=2)
                r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
            else
                r_hand_poseT = nan(6,1);
            end
            if(goal_type_flags.lh ~=2)
                l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];
            else
                l_hand_poseT = nan(6,1);
            end
            r_foot_poseT = [rfootT(1:3,:); repmat(rpy2quat(rfootT(4:6,1)),1,num_r_foot_pts)];
            l_foot_poseT = [lfootT(1:3,:); repmat(rpy2quat(lfootT(4:6,1)),1,num_l_foot_pts)];
            headT(1:3)=nan(3,1); % only orientation constraint for the head.
            head_poseT = [headT(1:3); rpy2quat(headT(4:6))];
            
            %======================================================================================================
            
            
            cost = getCostVector(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.quasiStaticFlag = true;
            ikoptions.shrinkFactor = 0.85;
            ikoptions.MajorIterationsLimit = 500;
            ikoptions.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
            ikoptions.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
            
              
            comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
            utorso_pose0_relaxed = utorso_pose0;
            %       utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0 = utorso_pose0(1:3);
            
            s = [0 1]; % normalized arc length index
            ks = ActionSequence();
            
            % kc_com = ActionKinematicConstraint(obj.r,0,[0;0;0],comgoal,[s(1),s(end)],'com');
            % ks = ks.addKinematicConstraint(kc_com);
            
            if(isempty(q_desired))
                r_hand_poseT_relaxed.min=r_hand_poseT-1e-3;
                r_hand_poseT_relaxed.max=r_hand_poseT+1e-3;
                l_hand_poseT_relaxed.min=l_hand_poseT-1e-3;
                l_hand_poseT_relaxed.max=l_hand_poseT+1e-3;
                r_foot_poseT_relaxed.min=r_foot_poseT-1e-3;
                r_foot_poseT_relaxed.max=r_foot_poseT+1e-3;
                l_foot_poseT_relaxed.min=l_foot_poseT-1e-3;
                l_foot_poseT_relaxed.max=l_foot_poseT+1e-3;
                head_poseT_relaxed.min = head_poseT-1e-3;
                head_poseT_relaxed.max = head_poseT+1e-3;
            else
                r_hand_poseT_relaxed.min=r_hand_poseT;
                r_hand_poseT_relaxed.max=r_hand_poseT;
                l_hand_poseT_relaxed.min=l_hand_poseT;
                l_hand_poseT_relaxed.max=l_hand_poseT;
                r_foot_poseT_relaxed.min=r_foot_poseT;
                r_foot_poseT_relaxed.max=r_foot_poseT;
                l_foot_poseT_relaxed.min=l_foot_poseT;
                l_foot_poseT_relaxed.max=l_foot_poseT;
                head_poseT_relaxed.min = head_poseT;
                head_poseT_relaxed.max = head_poseT;
            end
            
            
            % End State Constraints
            % Constraints for feet
            
            if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                if(obj.restrict_feet)
                    kc_rfoot = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose0,[s(1),s(end)],'rfoot0',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_rfoot);
                    kc_lfoot = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose0,[s(1),s(end)],'lfoot0',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_lfoot);
                else
                    kc_rfoot0 = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose0,[s(1),s(1)],'rfoot0',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_rfoot0);
                    kc_lfoot0 = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose0,[s(1),s(1)],'lfoot0',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_lfoot0);
                    kc_rfootT = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_poseT_relaxed,[s(end),s(end)],'rfootT',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_rfootT);
                    kc_lfootT = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_poseT_relaxed,[s(end),s(end)],'lfootT',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_lfootT);
                end
            end % end if(~obj.isBDIManipMode())
            
            
            % Constraints for hands
            kc_rhand0 = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose0,[s(1),s(1)],'rhand0');
            ks = ks.addKinematicConstraint(kc_rhand0);
            kc_rhandT = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_poseT_relaxed,[s(end),s(end)],'rhandT');
            ks = ks.addKinematicConstraint(kc_rhandT);
            kc_lhand0 = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose0,[s(1),s(1)],'lhand0');
            ks = ks.addKinematicConstraint(kc_lhand0);
            kc_lhandT = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_poseT_relaxed,[s(end),s(end)],'lhandT');
            ks = ks.addKinematicConstraint(kc_lhandT);
            
            % Constraints for head
            kc_head0 = ActionKinematicConstraint(obj.r,obj.head_body,[0;0;0],head_pose0,[s(1),s(1)],'head0');
            %                 ks = ks.addKinematicConstraint(kc_head0);
            kc_headT = ActionKinematicConstraint(obj.r, obj.head_body,[0;0;0],head_poseT_relaxed,[s(end),s(end)],'headT');
            %                 ks = ks.addKinematicConstraint(kc_headT);
            
            if(obj.restrict_feet)
                kc_pelvis = ActionKinematicConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose0,[s(1),s(end)],'pelvis');
                ks = ks.addKinematicConstraint(kc_pelvis);
            end
            % kc_torso = ActionKinematicConstraint(obj.r,obj.utorso_body,[0;0;0],utorso_pose0_relaxed,[s(1),s(end)],'utorso');
            % ks = ks.addKinematicConstraint(kc_torso);
            
            
            % Solve IK at final pose and pass as input to sequence search
            pert = [1e-3*ones(3,1); 1e-2*ones(4,1)];
            
            rhand_const.min = r_hand_poseT-repmat(pert,1,size(r_hand_poseT,2));
            rhand_const.max = r_hand_poseT+repmat(pert,1,size(r_hand_poseT,2));
            lhand_const.min = l_hand_poseT-repmat(pert,1,size(l_hand_poseT,2));
            lhand_const.max = l_hand_poseT+repmat(pert,1,size(l_hand_poseT,2));
            rfoot_const.min = r_foot_poseT-repmat(pert,1,size(r_foot_poseT,2));
            rfoot_const.max = r_foot_poseT+repmat(pert,1,size(r_foot_poseT,2));
            lfoot_const.min = l_foot_poseT-repmat(pert,1,size(l_foot_poseT,2));
            lfoot_const.max = l_foot_poseT+repmat(pert,1,size(l_foot_poseT,2));
            head_const.min = head_poseT-repmat(pert,1,size(head_poseT,2));
            head_const.max = head_poseT+repmat(pert,1,size(head_poseT,2));
        
            if(~isempty(head_gaze_target))
                head_const.type = 'gaze';
                head_const.gaze_target = head_gaze_target;
                head_const.gaze_conethreshold = pi/12;
                head_const.gaze_axis = [1;0;0];
            end
            if(~isempty(rhand_gaze_target))
                rhand_const.type = 'gaze';
                rhand_const.gaze_target = rhand_gaze_target;
                rhand_const.gaze_conethreshold = pi/18;
                rhand_const.gaze_axis = [1;0;0];
            end
            if(~isempty(lhand_gaze_target))
                lhand_const.type = 'gaze';
                lhand_const.gaze_target = lhand_gaze_target;
                lhand_const.gaze_conethreshold = pi/18;
                lhand_const.gaze_axis = [1;0;0];
            end
            %============================
            %       0,comgoal,...
            q_final_quess= q0;
            
            if(isempty(q_desired))
                q_start=q0;
                r_foot_pose0_static_contact = struct('max',r_foot_pose0,...
                    'min',r_foot_pose0,...
                    'contact_state',{ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    'contact_dist',{struct('min',zeros(1,num_r_foot_pts),'max',zeros(1,num_r_foot_pts))},...
                    'contact_affs',{ContactAffordance()});
                l_foot_pose0_static_contact = struct('max',l_foot_pose0,...
                    'min',l_foot_pose0,...
                    'contact_state',{ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    'contact_dist',{struct('min',zeros(1,num_l_foot_pts),'max',zeros(1,num_l_foot_pts))},...
                    'contact_affs',{ContactAffordance()});
                
                if(obj.planning_mode == 3)% teleop mode
                    kinsol = doKinematics(obj.r,q0);
                    rhand_pose = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
                    lhand_pose = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
                    head_pose = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
                    pelvis_pose = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
                    if(all(isnan(pelvis_pose0)))
                        pelvis_pose0 = pelvis_pose;
                    end
                    if(isstruct(rhand_const))
                        if(isfield(rhand_const,'max'))
                            if(all(isnan(rhand_const.max))&&all(isnan(rhand_const.min)))
                                rhand_const = struct('max',rhand_pose,'min',rhand_pose);
                            end
                        end
                    else
                        if(all(isnan(rhand_const)))
                            rhand_const = rhand_pose;
                        end
                    end
                    if(isstruct(lhand_const))
                        if(isfield(lhand_const,'max'))
                            if(all(isnan(lhand_const.max))&&all(isnan(lhand_const.min)))
                                lhand_const = struct('max',lhand_pose,'min',lhand_pose);
                            end
                        end
                    else
                        if(all(isnan(lhand_const)))
                            lhand_const = lhand_pose;
                        end
                    end
                    if(isstruct(head_const))
                        if(isfield(head_const,'max'))
                            if(all(isnan(head_const.max))&&all(isnan(head_const.min)))
                                head_const.max = head_pose;
                                head_const.min = head_pose;
                            end
                        end
                    else
                        if(all(isnan(head_const)))
                            head_const = head_pose;
                        end
                    end
                end
                
                if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                    if(obj.restrict_feet)
                        %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        [q_final_guess,snopt_info] = inverseKin(obj.r,q_start,...
                            obj.r_foot_body,r_foot_pts,r_foot_pose0_static_contact,...
                            obj.l_foot_body,l_foot_pts,l_foot_pose0_static_contact, ...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            obj.head_body,[0;0;0],head_const,...
                            ikoptions);
                    else
                        % if feet are not restricted then you need to add back pelvis constraint
                        [q_final_guess,snopt_info] = inverseKin(obj.r,q_start,...
                            obj.pelvis_body,[0;0;0],pelvis_pose0,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact, ...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact, ...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            obj.head_body,[0;0;0],head_const,...
                            ikoptions);
                    end
                else
                    ikoptions.quasiStaticFlag = false;
                    [q_final_guess,snopt_info] = inverseKin(obj.r,q_start,...
                        obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        obj.r_hand_body,[0;0;0],rhand_const, ...
                        obj.l_hand_body,[0;0;0],lhand_const,...
                        obj.head_body,[0;0;0],head_const,...
                        ikoptions);
                end % end if(~obj.isBDIManipMode())
                
                if(snopt_info >10)
                    warning('The IK fails at the end');
                    send_status(4,0,0,sprintf('snopt_info = %d. Reaching plan initial IK is not very good.',snopt_info));
                end
            else
                q_final_guess =q_desired;
                snopt_info = 0;
            end
            %============================
            
            s_breaks=[s(1) s(end)];
            q_breaks=[q0 q_final_guess];
            qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_final_guess]));
            
            
            if(obj.planning_mode == 1)
                % PERFORM IKSEQUENCE OPT
                ikseq_options.Q = diag(cost(1:getNumDOF(obj.r)));
                ikseq_options.Qa = eye(getNumDOF(obj.r));
                ikseq_options.Qv = eye(getNumDOF(obj.r));
                ikseq_options.nSample = obj.plan_cache.num_breaks-1;
                ikseq_options.qdotf.lb = zeros(obj.r.getNumDOF(),1);
                ikseq_options.qdotf.ub = zeros(obj.r.getNumDOF(),1);
                if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                    ikseq_options.quasiStaticFlag=true;
                else
                    ikseq_options.Qa = 0.05*eye(getNumDOF(obj.r));
                    ikseq_options.Qv = 0*eye(getNumDOF(obj.r));
                    ikseq_options.quasiStaticFlag=false;
                end
                ikseq_options.shrinkFactor = 0.9;
                ikseq_options.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
                ikseq_options.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
                
                ikseq_options.MajorIterationsLimit = 200;
                ikseq_options.qtraj0 = qtraj_guess;
                
                ikseq_options.q_traj_nom = ikseq_options.qtraj0; % Without this the cost function is never used
                %============================
                [s_breaks,q_breaks,qdos_breaks,qddos_breaks,snopt_info] = inverseKinSequence(obj.r,q0,0*q0,ks,ikseq_options);
                if(snopt_info > 10)
                    warning('The IK sequence fails');
                    send_status(4,0,0,sprintf('snopt_info = %d. The IK sequence fails.',snopt_info));
                end
                %============================
                xtraj = PPTrajectory(pchipDeriv(s_breaks,[q_breaks;qdos_breaks],[qdos_breaks;qddos_breaks]));
                xtraj = xtraj.setOutputFrame(obj.r.getStateFrame()); %#ok<*NASGU>
                
                qtraj_guess = PPTrajectory(spline(s_breaks,q_breaks));
            end
            
            % calculate end effectors breaks via FK.
            for brk =1:length(s_breaks),
                kinsol_tmp = doKinematics(obj.r,q_breaks(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
                ruarm_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_uarm_body,[0;0;0],2);
                luarm_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_uarm_body,[0;0;0],2);  
            end
            
            q = q_breaks(:,1);
            
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_lel =  sum(sqrt(sum(diff(luarm_breaks(1:3,:),1,2).^2,1)));
            s_total_rel =  sum(sqrt(sum(diff(ruarm_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),max(s_total_head,max(s_total_lel,s_total_rel)));
            s_total = max(s_total,0.01);
            
            res = 0.15; % 20cm res
            s= linspace(0,1,ceil(s_total/res)+1); % Must have two points atleast
            s = unique([s(:);s_breaks(:)]);
            % fine grained verification of COM constraints of fixed resolution.
            for i=2:length(s)
                si = s(i);
                q(:,i) =qtraj_guess.eval(si);
            end
            
            % update plan cache for keyframe adjustment engine
            obj.plan_cache.ks = ks; % Cache Boundary Constraints
            obj.plan_cache.s = s;
            obj.plan_cache.s_breaks = s_breaks;
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            if(obj.planning_mode==1)
              obj.plan_cache.quasiStaticFlag = ikseq_options.quasiStaticFlag;
            else
              obj.plan_cache.quasiStaticFlag = false;
            end

            % publish robot plan
            disp('Publishing plan...');
            xtraj = zeros(getNumStates(obj.r)+2,length(s));
            xtraj(1,:) = 0*s;
            xtraj(2,:) = 0*s;
            if(length(s_breaks)>obj.plan_cache.num_breaks)
                keyframe_inds = unique(round(linspace(1,length(s_breaks),obj.plan_cache.num_breaks)));
            else
                keyframe_inds =[1:length(s_breaks)];
            end
            
            for l = keyframe_inds,
                ind = find(s == s_breaks(l));
                xtraj(1,ind) = 1.0;
            end
            xtraj(3:getNumDOF(obj.r)+2,:) = q;
            snopt_info_vector = snopt_info*ones(1, size(xtraj,2));
            
            dqtraj=fnder(obj.plan_cache.qtraj,1); 
            sfine = linspace(s(1),s(end),50);
            Tmax_joints = max(max(abs(eval(dqtraj,sfine)),[],2))/obj.plan_cache.qdot_desired;
            Tmax_ee  = (s_total/obj.plan_cache.v_desired);
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            utime = now() * 24 * 60 * 60;
            % ignore the first state
            % ts = ts(2:end);
            % xtraj=xtraj(:,2:end);
            obj.plan_pub.publish(xtraj,ts,utime, snopt_info_vector);
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
