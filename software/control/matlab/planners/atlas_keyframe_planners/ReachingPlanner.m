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
        num_breaks
        hand_space; % A HandWorkspace object, used to retrieve a seed for IK.
    end
    
    methods
        function obj = ReachingPlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode)
            obj = obj@KeyframePlanner(r,atlas,lhand_frame,rhand_frame); % initialize the base class
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = atlas.getStateFrame.coordinates(1:getNumDOF(atlas));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            
            obj.num_breaks = 4;
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.restrict_feet=true;
            obj.planning_mode = 1;
            obj.hand_space = HandWorkspace([getenv('DRC_PATH'),'/control/matlab/data/HandWorkSpace.mat']);
            coords = obj.r.getStateFrame.coordinates;
            coords = coords(1:obj.r.getNumDOF);
        end
        
        function setPlanningMode(obj,val)
            obj.planning_mode  = val;
            if(val==3)
                obj.plan_cache.inTeleopMode = true; 
            else
                obj.plan_cache.inTeleopMode = false; 
            end
        end
        %-----------------------------------------------------------------------------------------------------------------
        function generateAndPublishReachingPlan(obj,varargin)
            
            switch nargin
                case 9
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    lidar_ee_goal = varargin{7};
                    goal_type_flags =varargin{8};
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,goal_type_flags);
                case 10
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    lidar_ee_goal = varargin{7};
                    goal_type_flags =varargin{8};
                    q_desired = varargin{9};
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,goal_type_flags,q_desired);
                otherwise
                    error('Incorrect usage of generateAndPublishReachingPlan in Reaching Planner. Undefined number of inputs.')
            end
        end
        
        function generateReachPlanToCompensateForCurrentSSE(obj,x0,mode)
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

            
            % Look for the last cached plan for commanded (qcmd).
            qcmd = obj.plan_cache.qtraj.eval(obj.plan_cache.s(end));
            kinsol_cmd = doKinematics(obj.r,qcmd);
            if(mode==drc.plan_adjust_mode_t.LEFT_HAND)
                lh_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_hand_body,[0;0;0],1);
                lh_des =lh_cmd;
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_HAND)
                rh_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_hand_body,[0;0;0],1);
                rh_des =rh_cmd;
            elseif(mode==drc.plan_adjust_mode_t.BOTH_HANDS)
                lh_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_hand_body,[0;0;0],1);
                lh_des =lh_cmd;
                rh_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_hand_body,[0;0;0],1);
                rh_des =rh_cmd;
            elseif(mode==drc.plan_adjust_mode_t.LEFT_FOOT)
                lf_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_foot_body,[0;0;0],1);
                lf_des =lf_cmd;
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_FOOT)
                rf_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_foot_body,[0;0;0],1);
                rf_des =rf_cmd;
            elseif(mode==drc.plan_adjust_mode_t.BOTH_FEET)
                lf_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_foot_body,[0;0;0],1);
                lf_des =lf_cmd;
                rf_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_foot_body,[0;0;0],1);
                rf_des =rf_cmd;
            elseif(mode==drc.plan_adjust_mode_t.ALL)
                lh_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_hand_body,[0;0;0],1);
                lh_des =lh_cmd;
                rh_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_hand_body,[0;0;0],1);
                rh_des =rh_cmd;
                lf_cmd = forwardKin(obj.r,kinsol_cmd,obj.l_foot_body,[0;0;0],1);
                lf_des =lf_cmd;
                rf_cmd = forwardKin(obj.r,kinsol_cmd,obj.r_foot_body,[0;0;0],1);
                rf_des =rf_cmd;
            end
            
            qcur = x0(1:getNumDOF(obj.r));
            kinsol_meas = doKinematics(obj.r,qcur);
            % get measured ee positions
            
            
            
            if(mode==drc.plan_adjust_mode_t.LEFT_HAND)
                lh_meas = forwardKin(obj.r,kinsol_meas,obj.l_hand_body,[0;0;0],1);
                lh_ee_goal=pose_shift(lh_des,lh_cmd,lh_meas,true);
                T_world_hand_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
                T_world_palm_l = T_world_hand_l*obj.T_hand_palm_l; 
                lh_ee_goal(1:3) = T_world_palm_l(1:3,4);
                lh_ee_goal(4:6) =rotmat2rpy(T_world_palm_l(1:3,1:3));               
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_HAND)
                rh_meas = forwardKin(obj.r,kinsol_meas,obj.r_hand_body,[0;0;0],1);
                rh_ee_goal=pose_shift(rh_des,rh_cmd,rh_meas,true);
                T_world_hand_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
                T_world_palm_r = T_world_hand_r*obj.T_hand_palm_r; 
                rh_ee_goal(1:3) = T_world_palm_r(1:3,4);
                rh_ee_goal(4:6) =rotmat2rpy(T_world_palm_r(1:3,1:3));  
            elseif(mode==drc.plan_adjust_mode_t.BOTH_HANDS)
                lh_meas = forwardKin(obj.r,kinsol_meas,obj.l_hand_body,[0;0;0],1);
                lh_ee_goal=pose_shift(lh_des,lh_cmd,lh_meas,true);                
                T_world_hand_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
                T_world_palm_l = T_world_hand_l*obj.T_hand_palm_l; 
                lh_ee_goal(1:3) = T_world_palm_l(1:3,4);
                lh_ee_goal(4:6) =rotmat2rpy(T_world_palm_l(1:3,1:3)); 
                
                rh_meas = forwardKin(obj.r,kinsol_meas,obj.r_hand_body,[0;0;0],1);
                rh_ee_goal=pose_shift(rh_des,rh_cmd,rh_meas,true);
                T_world_hand_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
                T_world_palm_r = T_world_hand_r*obj.T_hand_palm_r; 
                rh_ee_goal(1:3) = T_world_palm_r(1:3,4);
                rh_ee_goal(4:6) =rotmat2rpy(T_world_palm_r(1:3,1:3)); 
            elseif(mode==drc.plan_adjust_mode_t.LEFT_FOOT)
                lf_meas = forwardKin(obj.r,kinsol_meas,obj.l_foot_body,[0;0;0],1);
                lf_ee_goal=pose_shift(lf_des,lf_cmd,lf_meas,true);
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_FOOT)
                rf_meas = forwardKin(obj.r,kinsol_meas,obj.r_foot_body,[0;0;0],1);
                rf_ee_goal=pose_shift(rf_des,rf_cmd,rf_meas,true);
            elseif(mode==drc.plan_adjust_mode_t.BOTH_FEET)
                lf_meas = forwardKin(obj.r,kinsol_meas,obj.l_foot_body,[0;0;0],1);
                lf_ee_goal=pose_shift(lf_des,lf_cmd,lf_meas,true);
                rf_meas = forwardKin(obj.r,kinsol_meas,obj.r_foot_body,[0;0;0],1);
                rf_ee_goal=pose_shift(rf_des,rf_cmd,rf_meas,true);
            elseif(mode==drc.plan_adjust_mode_t.ALL)
                lh_meas = forwardKin(obj.r,kinsol_meas,obj.l_hand_body,[0;0;0],1);
                lh_ee_goal=pose_shift(lh_des,lh_cmd,lh_meas,true);                
                T_world_hand_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
                T_world_palm_l = T_world_hand_l*obj.T_hand_palm_l; 
                lh_ee_goal(1:3) = T_world_palm_l(1:3,4);
                lh_ee_goal(4:6) =rotmat2rpy(T_world_palm_l(1:3,1:3));                 
                rh_meas = forwardKin(obj.r,kinsol_meas,obj.r_hand_body,[0;0;0],1);
                rh_ee_goal=pose_shift(rh_des,rh_cmd,rh_meas,true);
                T_world_hand_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
                T_world_palm_r = T_world_hand_r*obj.T_hand_palm_r; 
                rh_ee_goal(1:3) = T_world_palm_r(1:3,4);
                rh_ee_goal(4:6) =rotmat2rpy(T_world_palm_r(1:3,1:3));                
                lf_meas = forwardKin(obj.r,kinsol_meas,obj.l_foot_body,[0;0;0],1);
                lf_ee_goal=pose_shift(lf_des,lf_cmd,lf_meas,true);
                rf_meas = forwardKin(obj.r,kinsol_meas,obj.r_foot_body,[0;0;0],1);
                rf_ee_goal=pose_shift(rf_des,rf_cmd,rf_meas,true);
            end
            
            previous_planning_mode = obj.planning_mode;
            setPlanningMode(obj,3); % set to teleop mode
            runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,goal_type_flags);
            setPlanningMode(obj,previous_planning_mode);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function runOptimization(obj,varargin)
            
            obj.plan_cache.clearCache();
            obj.plan_cache.num_breaks = obj.num_breaks;
            
            q_desired= [];
            rh_ee_goal= [];
            lh_ee_goal= [];
            rf_ee_goal= [];
            lf_ee_goal= [];
            h_ee_goal = [];
            lidar_ee_goal = [];
            goal_type_flags.lh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
            goal_type_flags.rh = 0;
            goal_type_flags.h  = 0;
            goal_type_flags.lf = 0;
            goal_type_flags.rf = 0;
            goal_type_flags.lidar = 0;
            switch nargin
                case 9
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    lidar_ee_goal = varargin{7};
                    goal_type_flags= varargin{8};
                case 10
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    lidar_ee_goal = varargin{7};
                    goal_type_flags= varargin{8};
                    q_desired = varargin{9};
                otherwise
                    error('Incorrect usage of runOptimization in Manip Planner. Undefined number of vargin.')
            end
            
            
            disp('Generating plan...');
            send_status(3,0,0,'Generating  plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            obj.checkPosture(q0);
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            [joint_lb_tmp,joint_ub_tmp] = obj.r.getJointLimits();
            q0_bound = min([q0 joint_ub_tmp],[],2);
            q0_bound = max([q0_bound joint_lb_tmp],[],2);
            
            if(obj.isBDIManipMode())
              % Add the joint constraints on the lower bodies in the BDI manip mode, to guarantee
              % that lower joints are fixed
              coords = obj.r.getStateFrame.coordinates();
              coords = coords(1:obj.r.getNumDOF());
              joint_idx = (1:obj.r.getNumDOF())';
              lower_fixed_joint_idx = joint_idx(cellfun(@(s) ~isempty(strfind(s,'leg')) | ~isempty(strfind(s,'base')),coords));
              lower_fixed_posture_constraint = PostureConstraint(obj.r);
              lower_fixed_posture_constraint = lower_fixed_posture_constraint.setJointLimits(lower_fixed_joint_idx,...
              q0_bound(lower_fixed_joint_idx),q0_bound(lower_fixed_joint_idx));
              if(any(q0_bound<joint_lb_tmp) || any(q0_bound>joint_ub_tmp))
                 error('Joint limit not satisfied');
              end
            else
              lower_fixed_posture_constraint = PostureConstraint(obj.r);
            end
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0_bound);
            r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            l_foot_pts = [0;0;0];
            r_foot_pts = [0;0;0];
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,2);
            
            
            % compute fixed COM goal
            gc = contactPositions(obj.r,q0_bound);
            k = convhull(gc(1:2,:)');
            com0 = getCOM(obj.r,kinsol);
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
                    T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
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
                    T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
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
                headT = nan(6,1);
            else
              if(goal_type_flags.h ~= 2)
                error('Currently we only set head ee goal when when there is a head gaze constraint');
              end
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
            lidar_gaze_target = [];
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
            if(goal_type_flags.lidar == 2)
              lidar_gaze_target = lidar_ee_goal(1:3);
            end
            %===========================================================================
            
            if(goal_type_flags.rh ~=2)
                r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
            else
                r_hand_poseT = nan(7,1);
            end
            if(goal_type_flags.lh ~=2)
                l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];
            else
                l_hand_poseT = nan(7,1);
            end
            if(goal_type_flags.h ~= 2)
              head_poseT = [headT(1:3);rpy2quat(headT(4:6))];
            else
              head_poseT = nan(7,1);
            end
            lhand_poseT_isnan = all(isnan(l_hand_poseT));
            rhand_poseT_isnan = all(isnan(r_hand_poseT));
            head_poseT_isnan = all(isnan(head_poseT));
            r_foot_poseT = [rfootT(1:3,:); repmat(rpy2quat(rfootT(4:6,1)),1,num_r_foot_pts)];
            l_foot_poseT = [lfootT(1:3,:); repmat(rpy2quat(lfootT(4:6,1)),1,num_l_foot_pts)];


            %======================================================================================================
            
            
            cost = getCostVector(obj);
            
            ikoptions = IKoptions(obj.r);
            ikoptions = ikoptions.setQ(diag(cost(1:getNumDOF(obj.r))));
            ik_qnom = q0_bound;
            qsc = QuasiStaticConstraint(obj.r);
            qsc = qsc.setActive(true);
            qsc = qsc.setShrinkFactor(0.85);
            ikoptions = ikoptions.setDebug(true);
            ikoptions = ikoptions.setMajorIterationsLimit(500);
            
            comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
            utorso_pose0_relaxed = utorso_pose0;
            %       utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0 = utorso_pose0(1:3);
            
            
            
            s = [0 1]; % normalized arc length index
            
            tol=1e-3;
            [rhand_poseT_min,rhand_poseT_max,rhand_poseT_quat,rhand_poseT_tol] = parsePoseT(obj,r_hand_poseT,tol,rhand_poseT_isnan);
            [lhand_poseT_min,lhand_poseT_max,lhand_poseT_quat,lhand_poseT_tol] = parsePoseT(obj,l_hand_poseT,tol,lhand_poseT_isnan);
            [rfoot_poseT_min,rfoot_poseT_max,rfoot_poseT_quat,rfoot_poseT_tol] = parsePoseT(obj,r_foot_poseT,tol,false);
            [lfoot_poseT_min,lfoot_poseT_max,lfoot_poseT_quat,lfoot_poseT_tol] = parsePoseT(obj,l_foot_poseT,tol,false);
            [head_poseT_min,head_poseT_max,head_poseT_quat,head_poseT_tol] = parsePoseT(obj,head_poseT,tol,head_poseT_isnan);
            
            
            % End State Constraints
            % Constraints for feet
            
            iktraj_lhand_constraint = {};
            iktraj_rhand_constraint = {};
            iktraj_lfoot_constraint = {};
            iktraj_rfoot_constraint = {};
            iktraj_pelvis_constraint = {};
            iktraj_head_constraint = {};
            if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                if(obj.restrict_feet)
                    rfoot_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose0,0,0,[s(1),s(end)]);
                    lfoot_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose0,0,0,[s(1),s(end)]);
                    iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];
                    iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];
                else
                    rfoot_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose0,0,0,[s(1),s(1)]);
                    lfoot_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose0,0,0,[s(1),s(1)]);
                    iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];
                    iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];
                    
                    iktraj_rfoot_constraint = [iktraj_rfoot_constraint,...
                        parse2PosQuatConstraint(obj.r,obj.r_foot_body,r_foot_pts,...
                        [(rfoot_poseT_min+rfoot_poseT_max)/2;rfoot_poseT_quat],...
                        (rfoot_poseT_max-rfoot_poseT_min)/2,r_foot_poseT_tol,[s(end),s(end)])];
                    iktraj_lfoot_constraint = [iktraj_lfoot_constraint,...
                        parse2PosQuatConstraint(obj.r,obj.l_foot_body,l_foot_pts,...
                        [(lfoot_poseT_min+lfoot_poseT_max)/2;lfoot_poseT_quat],...
                        (lfoot_poseT_max-lfoot_poseT_min)/2,l_foot_poseT_tol,[s(end),s(end)])];
                    
                end
                qsc = qsc.addContact(obj.l_foot_body,l_foot_contact_pts,obj.r_foot_body,r_foot_contact_pts);
            end % end if(~obj.isBDIManipMode())
            
            
            % Constraints for hands
            quat_Tol=sind(2).^2;
            rhand_constraint0 = {WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose0(1:3,:),r_hand_pose0(1:3,:),[s(1),s(1)]),...
                WorldQuatConstraint(obj.r,obj.r_hand_body,r_hand_pose0(4:7,1),quat_Tol,[s(1) s(1)])};
            lhand_constraint0 = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose0(1:3,:),l_hand_pose0(1:3,:),[s(1),s(1)]),...
                WorldQuatConstraint(obj.r,obj.l_hand_body,l_hand_pose0(4:7,1),quat_Tol,[s(1) s(1)])};
            iktraj_rhand_constraint = [iktraj_rhand_constraint,rhand_constraint0];
            iktraj_lhand_constraint = [iktraj_lhand_constraint,lhand_constraint0];
            if(~rhand_poseT_isnan)
                rhand_constraintT = {WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_poseT_min,rhand_poseT_max,[s(end),s(end)]),...
                    WorldQuatConstraint(obj.r,obj.r_hand_body,rhand_poseT_quat,quat_Tol,[s(end),s(end)])};
                iktraj_rhand_constraint = [iktraj_rhand_constraint,rhand_constraintT];
            end
            if(~lhand_poseT_isnan)
                lhand_constraintT = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_poseT_min,lhand_poseT_max,[s(end),s(end)]),...
                    WorldQuatConstraint(obj.r,obj.l_hand_body,lhand_poseT_quat,quat_Tol,[s(end),s(end)])};
                iktraj_lhand_constraint = [iktraj_lhand_constraint,lhand_constraintT];
            end
            
            % Constraints for head
            head_constraint0 = {WorldPositionConstraint(obj.r,obj.head_body,[0;0;0],head_pose0(1:3,:),head_pose0(1:3,:),[s(1),s(1)]),...
                WorldQuatConstraint(obj.r,obj.head_body,head_pose0(4:7,1),quat_Tol,[s(1) s(1)])};
            if(~head_poseT_isnan)
                head_constraintT = {WorldPositionConstraint(obj.r,obj.head_body,[0;0;0],head_poseT_min,head_poseT_max,[s(end),s(end)]),...
                    WorldQuatConstraint(obj.r,obj.head_body,head_poseT_quat,head_poseT_tol,[s(end),s(end)])};
            end
            
            
            if(obj.restrict_feet)
                pelvis_quat_Tol=sind(0.01).^2;
                pelvis_constraint = {WorldPositionConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose0(1:3,:),pelvis_pose0(1:3,:),[s(1),s(end)]),...
                    WorldQuatConstraint(obj.r,obj.pelvis_body,pelvis_pose0(4:7,1),pelvis_quat_Tol,[s(1) s(end)])};
                iktraj_pelvis_constraint = [iktraj_pelvis_constraint,pelvis_constraint];
            end
            
            % Solve IK at final pose and pass as input to sequence search
            pert = [1e-3*ones(3,1); 1e-2*ones(4,1)];
            
            
            lhand_min = l_hand_poseT-repmat(pert,1,size(l_hand_poseT,2));
            lhand_max = l_hand_poseT+repmat(pert,1,size(l_hand_poseT,2));
            rfoot_min = r_foot_poseT-repmat(pert,1,size(r_foot_poseT,2));
            rfoot_max = r_foot_poseT+repmat(pert,1,size(r_foot_poseT,2));
            lfoot_min = l_foot_poseT-repmat(pert,1,size(l_foot_poseT,2));
            lfoot_max = l_foot_poseT+repmat(pert,1,size(l_foot_poseT,2));
            head_min = head_poseT-repmat(pert,1,size(head_poseT,2));
            head_max = head_poseT+repmat(pert,1,size(head_poseT,2));
            tspan = [1 1];
            if(~rhand_poseT_isnan)
                rhand_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_poseT,1e-3,1e-4,tspan);
            else
                rhand_constraint = {};
            end
            
            if(~lhand_poseT_isnan)
                lhand_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_poseT,1e-3,1e-4,tspan);
            else
                lhand_constraint = {};
            end
            
            if(~head_poseT_isnan)
                head_constraint = parse2PosQuatConstraint(obj.r,obj.head_body,[0;0;0],head_poseT,1e-3,1e-4,tspan);
            else
                head_constraint = {};
            end
            rfoot_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_poseT,1e-3,1e-4,tspan);
            lfoot_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_poseT,1e-3,1e-4,tspan);
            
            
            if(~isempty(head_gaze_target))
                head_constraint = [head_constraint,{WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,head_gaze_target,obj.h_camera_origin,obj.head_gaze_tol)}];
                iktraj_head_constraint = [iktraj_head_constraint,head_constraint];
            end
            if(~isempty(rhand_gaze_target))
                rhand_constraint = [rhand_constraint,{WorldGazeTargetConstraint(obj.r,obj.r_hand_body,obj.rh_gaze_axis,rhand_gaze_target,obj.rh_camera_origin,obj.hand_gaze_tol)}];
                iktraj_rhand_constraint = [iktraj_rhand_constraint,rhand_constraint];
            end
            if(~isempty(lhand_gaze_target))
                lhand_constraint = [lhand_constraint,{WorldGazeTargetConstraint(obj.r,obj.l_hand_body,obj.lh_gaze_axis,lhand_gaze_target,obj.lh_camera_origin,obj.hand_gaze_tol)}];
                iktraj_lhand_constraint = [iktraj_lhand_constraint,lhand_constraint];
            end
            
            if(~isempty(lidar_gaze_target))
              new_head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,lidar_gaze_target,obj.h_camera_origin,obj.lidar_gaze_tol)};
              head_constraint = [head_constraint,new_head_constraint];
              iktraj_head_constraint = [iktraj_head_constraint,new_head_constraint];
            end
            
            %============================
            %       0,comgoal,...
            q_final_guess= q0_bound;
            
            if(isempty(q_desired))
                q_start=q0_bound;
                rfoot_pose0_constraint = {WorldPositionConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose0(1:3,:),r_foot_pose0(1:3,:),tspan),...
                    WorldQuatConstraint(obj.r,obj.r_foot_body,r_foot_pose0(4:7,1),0,tspan)};
                lfoot_pose0_constraint = {WorldPositionConstraint(obj.r,obj.l_foot_body,r_foot_pts,l_foot_pose0(1:3,:),l_foot_pose0(1:3,:),tspan),...
                    WorldQuatConstraint(obj.r,obj.l_foot_body,l_foot_pose0(4:7,1),0,tspan)};
                qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);
                
                if(obj.planning_mode == 3)% teleop mode
                    kinsol = doKinematics(obj.r,q0_bound);
                    rhand_pose = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
                    lhand_pose = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
                    head_pose = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
                    pelvis_pose = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
                    
                    tspan = [1 1];
                    if(all(isnan(pelvis_pose0)))
                        pelvis_constraint = [pelvis_constraint,parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose,0,0,tspan)];
                    end
                    if(rhand_poseT_isnan)
                        rhand_constraint = [rhand_constraint,parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_pose,0,0,tspan)];
                    end
                    if(lhand_poseT_isnan)
                        lhand_constraint = [lhand_constraint,parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_pose,0,0,tspan)];
                    end
                    if(head_poseT_isnan)
                        head_constraint = [head_constraint,parse2PosQuatConstraint(obj.r,obj.head_body,[0;0;0],head_pose,0,0,tspan)];
                    end
                end
                
                
                hand_joint_idx = [obj.lhand2robotFrameIndMap(obj.lhand2robotFrameIndMap <= obj.r.getNumDOF);...
                  obj.rhand2robotFrameIndMap(obj.rhand2robotFrameIndMap <= obj.r.getNumDOF)];
                reaching_joint_cnst = obj.joint_constraint;
                reaching_joint_cnst = reaching_joint_cnst.setJointLimits(hand_joint_idx,...
                  q0_bound(hand_joint_idx),...
                  q0_bound(hand_joint_idx));
                if(obj.planning_mode == 4)
                  if(isempty(rhand_constraint))
                    reaching_joint_cnst = reaching_joint_cnst.setJointLimits(obj.r_arm_joint_ind,...
                      q0_bound(obj.r_arm_joint_ind),...
                      q0_bound(obj.r_arm_joint_ind));
                  end
                  if(isempty(lhand_constraint))
                    reaching_joint_cnst = reaching_joint_cnst.setJointLimits(obj.l_arm_joint_ind,...
                      q0_bound(obj.l_arm_joint_ind),...
                      q0_bound(obj.l_arm_joint_ind));
                  end
                end
                if(obj.isBDIManipMode())
                  reaching_joint_cnst = reaching_joint_cnst.setJointLimits(lower_fixed_joint_idx,q0_bound(lower_fixed_joint_idx),q0_bound(lower_fixed_joint_idx));
                end
                
                
                total_ik_attempt = 30;
                
                
                if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                    if(obj.restrict_feet)
                        %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        [q_final_guess,snopt_info,infeasible_constraint] = inverseKinRepeatSearch(obj.r,total_ik_attempt,q_start,ik_qnom,...
                            rfoot_pose0_constraint{:},lfoot_pose0_constraint{:},...
                            rhand_constraint{:},lhand_constraint{:},head_constraint{:},...
                            reaching_joint_cnst,qsc,ikoptions);
                    else
                        % if feet are not restricted then you need to add back pelvis constraint
                        [q_final_guess,snopt_info,infeasible_constraint] = inverseKinRepeatSearch(obj.r,total_ik_attempt,q_start,ik_qnom,...
                            pelvis_constraint{:},...
                            rfoot_pose0_constraint{:},lfoot_pose0_constraint{:},...
                            rhand_constraint{:},lhand_constraint{:},head_constraint{:},...
                            reaching_joint_cnst,qsc,ikoptions);
                    end
                else
                    [q_final_guess,snopt_info,infeasible_constraint] = inverseKinRepeatSearch(obj.r,total_ik_attempt,q_start,ik_qnom,...
                        rhand_constraint{:},lhand_constraint{:},head_constraint{:},...
                        reaching_joint_cnst,ikoptions);
                end % end if(~obj.isBDIManipMode())
                
                  
                
                
                if(snopt_info >10)
                    % this warning is at an intermediate point in the planning
                    % it is not an indication that the final plan is in violation
                    
                    send_msg = sprintf('snopt_info = %d. Reaching plan initial IK is not very good.',snopt_info);
                    if(obj.planning_mode == 2 || obj.planning_mode == 3)
                      send_status(4,0,0,send_msg);
                    end
                    display(infeasibleConstraintMsg(infeasible_constraint));
                end
            else
                q_final_guess =q_desired;
                snopt_info = 0;
            end
            %============================
            

            qtraj_guess = PPTrajectory(spline([s(1) s(end)],[zeros(obj.r.getNumDOF,1) q0_bound q_final_guess zeros(obj.r.getNumDOF,1)]));
%             collision_constraint = AllBodiesClosestDistanceConstraint(obj.r,0.01,1e3,[s(1) 0.01*s(1)+0.99*s(end)]);
            iktraj_tbreaks = linspace(s(1),s(end),obj.plan_cache.num_breaks);
            if(obj.planning_mode == 1 || obj.planning_mode == 4)
                % PERFORM inverseKinTraj OPT
                iktraj_options = IKoptions(obj.r);
                iktraj_options = iktraj_options.setDebug(true);
                iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
                iktraj_options = iktraj_options.setQa(0.05*eye(getNumDOF(obj.r)));
                iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
                iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1)); % upper and lower bnd on velocity.
                if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                    qsc = qsc.setActive(true);
                else
                    qsc = qsc.setActive(false);
                end
                qsc = qsc.setShrinkFactor(0.9);
                iktraj_options = iktraj_options.setMajorIterationsLimit(300);
                
                %============================
                iktraj_t_verify = linspace(iktraj_tbreaks(1),iktraj_tbreaks(end),20);
                [xtraj,snopt_info,infeasible_constraint] = inverseKinTrajWcollision(obj.r,obj.collision_check,iktraj_t_verify,...
                    iktraj_tbreaks,qtraj_guess,qtraj_guess,...
                    iktraj_head_constraint{:},...
                    iktraj_rhand_constraint{:},iktraj_lhand_constraint{:},...
                    iktraj_rfoot_constraint{:},iktraj_lfoot_constraint{:},...
                    iktraj_pelvis_constraint{:},reaching_joint_cnst,qsc,...
                    lower_fixed_posture_constraint,...
                    iktraj_options);
                if(snopt_info > 10)
                    warning('The IK traj fails');
                    send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
                    send_status(4,0,0,send_msg);
                    display(infeasibleConstraintMsg(infeasible_constraint));
                end
                %============================
                
                xtraj = xtraj.setOutputFrame(obj.r.getStateFrame()); %#ok<*NASGU>
                
                s_breaks = iktraj_tbreaks;
                x_breaks = xtraj.eval(s_breaks);
                q_breaks = x_breaks(1:obj.r.getNumDOF,:);
                qdot0 = x_breaks(obj.r.getNumDOF+(1:obj.r.getNumDOF),1);
                qdotf = x_breaks(obj.r.getNumDOF+(1:obj.r.getNumDOF),end);
                qtraj_guess = PPTrajectory(spline(s_breaks,[qdot0 q_breaks qdotf]));
            elseif(obj.planning_mode == 2 || obj.planning_mode == 3)
              s_breaks = iktraj_tbreaks;
              q_breaks = qtraj_guess.eval(s_breaks);
              qdot0 = zeros(obj.r.getNumDOF,1);
              qdotf = zeros(obj.r.getNumDOF,1);
            end
            
            
            Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
            s_total = Tmax_ee*obj.plan_cache.v_desired;
            
            s = linspace(0,1,max(ceil(s_total/obj.plan_arc_res)+1,5)); % Must have two points atleast
            s = unique([s(:);s_breaks(:)]);
            
            % fine grained verification of COM constraints of fixed resolution.
            q = q_breaks(:,1);
            for i=2:length(s)
                si = s(i);
                q(:,i) =qtraj_guess.eval(si);
            end
            
            % update plan cache for keyframe adjustment engine
            if(obj.planning_mode == 1 || obj.planning_mode == 4)
                obj.plan_cache.lfoot_constraint_cell = iktraj_lfoot_constraint;
                obj.plan_cache.rfoot_constraint_cell = iktraj_rfoot_constraint;
                obj.plan_cache.lhand_constraint_cell = iktraj_lhand_constraint;
                obj.plan_cache.rhand_constraint_cell = iktraj_rhand_constraint;
                obj.plan_cache.pelvis_constraint_cell = iktraj_pelvis_constraint;
            else
                obj.plan_cache.num_breaks = 2;
                obj.plan_cache.lfoot_constraint_cell = lfoot_pose0_constraint;
                obj.plan_cache.rfoot_constraint_cell = rfoot_pose0_constraint;
                obj.plan_cache.lhand_constraint_cell = lhand_constraint;
                obj.plan_cache.rhand_constraint_cell = rhand_constraint;
                obj.plan_cache.pelvis_constraint_cell = pelvis_constraint;
                if(obj.planning_mode==3)
                  obj.plan_cache.inTeleopMode = true; % make sure this is set.
                end
            end
            obj.plan_cache.s = s;
            obj.plan_cache.s_breaks = s_breaks;
            obj.plan_cache.qtraj = PPTrajectory(spline(s, [qdot0 q qdotf]));
            if(obj.planning_mode==1)
                obj.plan_cache.qsc = obj.plan_cache.qsc.setActive(qsc.active);
            else
                obj.plan_cache.qsc = obj.plan_cache.qsc.setActive(false);
            end
            obj.plan_cache.qsc = obj.plan_cache.qsc.setShrinkFactor(0.9);
            if(~obj.isBDIManipMode())
                obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.l_foot_body,l_foot_contact_pts,obj.r_foot_body,r_foot_contact_pts);
            end % end if(~obj.isBDIManipMode())
            
            % publish robot plan
            disp('Publishing plan...');
            nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
            xtraj_atlas = zeros(2+2*nq_atlas,length(s));
            
            xtraj_atlas(1,:) = 0*s;
            xtraj_atlas(2,:) = 0*s;
            if(length(s_breaks)>obj.plan_cache.num_breaks)
                keyframe_inds = unique(round(linspace(1,length(s_breaks),obj.plan_cache.num_breaks)));
            else
                keyframe_inds =[1:length(s_breaks)];
            end
            
            for l = keyframe_inds,
                xtraj_atlas(1,s == s_breaks(l)) = 1.0;
            end
            xtraj_atlas(2+(1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
            snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));
            
            Tmax_joints=obj.getTMaxForMaxJointSpeed();
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            utime = get_timestamp_now();% equivalent to bot_timestamp_now();
            
            obj.plan_pub.publish(xtraj_atlas,ts,utime, snopt_info_vector);
            display(sprintf('Reaching planner ts %5.3f\n',ts(end)));
        end
        %-----------------------------------------------------------------------------------------------------------------
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
        %-----------------------------------------------------------------------------------------------------------------
        function [pos_min,pos_max,pose_quat,pose_tol] = parsePoseT(obj,pose,tol,pose_isnan)
            % A utility function to pase the pose and tol
            if(size(pose,1)~=7)
                error('pose must have 7 rows');
            end
            pos_min = pose(1:3,:)-tol;
            pos_max = pose(1:3,:)+tol;
            if(~pose_isnan)
                pose_quat = pose(4:7,1);
                pose_tol = tol;
            else
                pose_quat = [];
                pose_tol = [];
            end
        end
        
    end% end methods
end% end classdef
