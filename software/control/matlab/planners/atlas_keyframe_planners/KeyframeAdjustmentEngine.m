classdef KeyframeAdjustmentEngine < KeyframePlanner
    % USAGE
    % KeyframeAdjustmentEngine adjustmentEngine(r);
    % cache = XPlanner.getPlanCache();
    % adjustmentEngine.setPlanCache(cache);
    % adjustmentEngine.adjustAndPublishCachedPlan(varargin);
    % Note: EndPose is a special case plan.
    % Note: TODO: for wholebodyplans do piece wise iksequence between each
    % breakpoints (otherwise its too slow)
    properties
        plan_pub
        pose_pub
    end
    
    methods
        function obj = KeyframeAdjustmentEngine(r,atlas,lhand_frame,rhand_frame,hardware_mode)
            obj = obj@KeyframePlanner(r,atlas,lhand_frame,rhand_frame); % initialize the base class
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = atlas.getStateFrame.coordinates(1:getNumDOF(atlas));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names); % if endpose flag is set
            obj.plan_cache = KeyframePlanCache(r);
        end
        %-----------------------------------------------------------------------------------------------------------------
        function setPlanCache(obj,cache)
            obj.plan_cache  = cache;
        end
        %-----------------------------------------------------------------------------------------------------------------
        function adjustAndPublishCachedPlan(obj,x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags)
            runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags);
        end
        %-----------------------------------------------------------------------------------------------------------------
        
        function q_start = getFirstPostureOfCachedPlan(obj)
            q_start = obj.plan_cache.qtraj.eval(0);
        end
        %-----------------------------------------------------------------------------------------------------------------
        
        function q_start = adjustCachedPlanToCurrentStateAndGetFirstPosture(obj,x0,mode)
          if(obj.plan_cache.num_grasp_transitions>0)
              [xtraj,ts,snopt_info_vector,G] = adjustCachedPlanToCurrentPelvisPose(obj,x0,mode);
          else
              [xtraj,ts] = adjustCachedPlanToCurrentPelvisPose(obj,x0,mode);
          end    
          
          % Decide if a posture plan needs to prepended if the current robot state is not the same as the first frame in the cached plan.
          q0 = x0(1:getNumDOF(obj.r));
          q_start = obj.plan_cache.qtraj.eval(0);
        end
       
        %-----------------------------------------------------------------------------------------------------------------
        function [varargout]=adjustCachedPlanToCurrentPelvisPose(obj,x0,mode)
            rh_ee_constraint= [];
            lh_ee_constraint= [];
            rf_ee_constraint= [];
            lf_ee_constraint= [];
            h_ee_constraint = [];
            goal_type_flags.lh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
            goal_type_flags.rh = 0;
            goal_type_flags.h  = 0;
            goal_type_flags.lf = 0;
            goal_type_flags.rf = 0;
            
            % delete all pelvis constraints
            % Add the new pelvis constraint for all of time.
            q0 = x0(1:getNumDOF(obj.r));
            kinsol_tmp = doKinematics(obj.r,q0);
            pelvis_pose = forwardKin(obj.r,kinsol_tmp,obj.pelvis_body,[0;0;0],2);
            
            q_samples = zeros(obj.r.getNumDOF(),length(obj.plan_cache.s));
            for i =1:length(obj.plan_cache.s)
                q_samples(:,i) = obj.plan_cache.qtraj.eval(obj.plan_cache.s(i));
            end
            
            constraints = {};
            pos_tol=1e-5*ones(3,1);
            quat_tol=sind(1).^2;
            kinsol_start = doKinematics(obj.r,q_samples(:,1));
            if(mode==drc.plan_adjust_mode_t.LEFT_HAND)
                obj.removeBodyConstraints(obj.l_foot_body,[-inf,inf]);
                obj.removeBodyConstraints(obj.r_foot_body,[-inf,inf]);
                obj.removeBodyConstraints(obj.r_hand_body,[-inf,inf]);
                lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_pose(1:3)-pos_tol,lhand_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_hand_body,lhand_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_HAND)
                obj.removeBodyConstraints(obj.l_foot_body,[-inf,inf]);
                obj.removeBodyConstraints(obj.r_foot_body,[-inf inf]);
                obj.removeBodyConstraints(obj.l_hand_body,[-inf inf]);
                rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_pose(1:3)-pos_tol,rhand_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.r_hand_body,rhand_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.BOTH_HANDS)
                obj.removeBodyConstraints(obj.l_foot_body,[-inf inf]);
                obj.removeBodyConstraints(obj.r_foot_body,[-inf inf]);
                lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
                rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_pose(1:3)-pos_tol,lhand_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_hand_body,lhand_pose(4:7),0),...
                    WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_pose(1:3)-pos_tol,rhand_pose(1:3)),...
                    WorldQuatConstraint(obj.r,obj.r_hand_body,rhand_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.LEFT_FOOT)
                obj.removeBodyConstraints(obj.l_hand_body,[-inf inf]);
                obj.removeBodyConstraints(obj.r_hand_body,[-inf inf]);
                obj.removeBodyConstraints(obj.r_foot_body,[-inf inf]);
                lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.l_foot_body,[0;0;0],lfoot_pose(1:3)-pos_tol,lfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_foot_body,lfoot_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_FOOT)
                obj.removeBodyConstraints(obj.l_hand_body,[-inf inf]);
                obj.removeBodyConstraints(obj.r_hand_body,[-inf inf]);
                obj.removeBodyConstraints(obj.l_foot_body,[-inf inf]);
                rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.r_foot_body,[0;0;0],rfoot_pose(1:3)-pos_tol,rfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.r_foot_body,rfoot_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.BOTH_FEET)
                obj.removeBodyConstraints(obj.l_hand_body,[-inf inf]);
                obj.removeBodyConstraints(obj.r_hand_body,[-inf inf]);
                lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
                rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.l_foot_body,[0;0;0],lfoot_pose(1:3)-pos_tol,lfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_foot_body,lfoot_pose(4:7),quat_tol),...
                    WorldPositionConstraint(obj.r,obj.r_foot_body,[0;0;0],rfoot_pose(1:3)-pos_tol,rfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.r_foot_body,rfoot_pose(4:7),quat_tol)};
            elseif(mode==drc.plan_adjust_mode_t.ALL)
                % dont remove anything
                lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
                rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
                lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
                rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
                constraints = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_pose(1:3)-pos_tol,lhand_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_hand_body,lhand_pose(4:7),quat_tol),...
                    WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_pose(1:3)-pos_tol,rhand_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.r_hand_body,rhand_pose(4:7),quat_tol),...
                    WorldPositionConstraint(obj.r,obj.l_foot_body,[0;0;0],lfoot_pose(1:3)-pos_tol,lfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.l_foot_body,lfoot_pose(4:7),quat_tol),...
                    WorldPositionConstraint(obj.r,obj.r_foot_body,[0;0;0],rfoot_pose(1:3)-pos_tol,rfoot_pose(1:3)+pos_tol),...
                    WorldQuatConstraint(obj.r,obj.r_foot_body,rfoot_pose(4:7),quat_tol)};
            end
            
            if(obj.plan_cache.isPointWiseIK== false)
                %============================
                cost = getCostVector(obj);
                ikoptions = IKoptions(obj.r);
                ikoptions = ikoptions.setDebug(true);
                ikoptions = ikoptions.setQ(diag(cost(1:getNumDOF(obj.r))));
                ik_qnom = q0;
                obj.plan_cache.qsc = obj.plan_cache.qsc.setActive(false);
                obj.plan_cache.qsc = obj.plan_cache.qsc.setShrinkFactor(0.85);
                ikoptions = ikoptions.setMajorIterationsLimit(500);
                %joint_constraint = PostureConstraint(obj.r);
                constraints = [constraints,{WorldPositionConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose(1:3),pelvis_pose(1:3)),...
                    WorldQuatConstraint(obj.r,obj.pelvis_body,pelvis_pose(4:7),0)}];
                [q_first,snopt_info,infeasible_constraint] = inverseKin(obj.r,q0,q0,constraints{:},obj.joint_constraint,obj.plan_cache.qsc,ikoptions);
                if(snopt_info > 10)
                    warning('The IK sequence fails');
                    send_msg = sprintf('snopt_info == %d.  IK failed to adjust first pose.\n %s',snopt_info,infeasibleConstraintMsg(infeasible_constraint));
                    send_status(4,0,0,send_msg);
                end
                q_samples(:,1) =  q_first;
                %============================
                % have to adjust the q at time 0 with a IK manually to the desired pelvis pose.
                % Ik Sequence by design does not modify the first posture at time zero.
                obj.plan_cache.qtraj = PPTrajectory(spline(obj.plan_cache.s,q_samples));
            end
            
            % delete old and add new
            tol = [1e-6*ones(3,1);1e-6*ones(4,1)];
            if(~obj.isBDIManipMode())
                 % remove old feet constraints and add new
                obj.removeBodyConstraints(obj.l_foot_body,[-inf,inf]);
                obj.removeBodyConstraints(obj.r_foot_body,[-inf,inf]);
                kinsol_tmp = doKinematics(obj.r,q0);
                lfoot_pose0 = forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                lfoot_pose.max = lfoot_pose0+tol(:);
                lfoot_pose.min = lfoot_pose0-tol(:);
                obj.cacheLFootPose([0 1],lfoot_pose);
                
                rfoot_pose0 = forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                rfoot_pose.max = rfoot_pose0+tol(:);
                rfoot_pose.min = rfoot_pose0-tol(:);
                obj.cacheRFootPose([0 1],rfoot_pose);
            else
                % remove old pelvis constraint and add new
                obj.plan_cache.pelvis_constraint_cell = {};            
                pose.max= pelvis_pose+tol;
                pose.min= pelvis_pose-tol;
                obj.cachePelvisPose([0 1],pose);    
            end
            
            pelvis_constraint = [];
            com_constraint = [];
            
            if(nargout==0)
                runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags);
            elseif (nargout==4)
                [xtraj,ts,snopt_info_vector,G]=runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags);
                varargout{1}=xtraj;  varargout{2}=ts;
                varargout{3}=snopt_info_vector;
                varargout{4}= G;
            elseif (nargout==2)
                [xtraj,ts]=runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags);
                varargout{1}=xtraj;  varargout{2}=ts;
            end
        end
        %-----------------------------------------------------------------------------------------------------------------
        function setCacheViaPlanMsg(obj,xtraj,ts,grasptransitions,logictraj)
 
            obj.plan_cache.clearCache();
  
            s = linspace(0,1,length(ts));
            s_breaks = s((logictraj(1,:)==1));
            if(~(isempty(s_breaks)))
                obj.plan_cache.isPointWiseIK= false;
            else
                obj.plan_cache.isPointWiseIK= true;
            end
            grasp_transition_breaks = s((logictraj(2,:)==1));
            
            obj.plan_cache.s = s;            
            
            obj.plan_cache.isEndPose = false;
            obj.plan_cache.num_breaks = sum(logictraj(1,:));
            obj.plan_cache.s_breaks = s_breaks;
            obj.plan_cache.grasp_transition_breaks = grasp_transition_breaks;
            obj.plan_cache.num_grasp_transitions = size(grasptransitions,2);%sum(logictraj(2,:));
            obj.plan_cache.grasp_transition_states = grasptransitions;
            obj.plan_cache.qtraj = PPTrajectory(spline(s,xtraj(1:getNumDOF(obj.r),:)));
            if(~obj.isBDIManipMode()) 
                obj.plan_cache.obj.plan_cache.qsc = obj.plan_cache.setActive(true);
            else
                obj.plan_cache.obj.plan_cache.qsc = obj.plan_cache.setActive(false);
            end
            
            % s = linspace(0,1,2*length(ts));
            if(obj.plan_cache.isPointWiseIK==true)
                s_breaks = s;
            end
            nq = obj.r.getNumDOF();
            q_breaks = zeros(nq,length(s_breaks));
            rhand_breaks = zeros(7,length(s_breaks));
            lhand_breaks = zeros(7,length(s_breaks));
            rfoot_breaks = zeros(7,length(s_breaks));
            lfoot_breaks = zeros(7,length(s_breaks));
            
            tol = [1e-6*ones(3,1);1e-4*ones(4,1)];
            for brk =1:length(s_breaks),
                q_breaks(:,brk) = obj.plan_cache.qtraj.eval(s_breaks(brk));
                kinsol_tmp = doKinematics(obj.r,q_breaks(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                %if((brk==1)||brk==length(s_breaks))
                s_val = s_breaks(brk);
                lhand_pose.max = lhand_breaks(:,brk)+tol(:);
                lhand_pose.min = lhand_breaks(:,brk)-tol(:);
                obj.cacheLHandPose([s_val s_val],lhand_pose);
                
                rhand_pose.max = rhand_breaks(:,brk)+tol(:);
                rhand_pose.min = rhand_breaks(:,brk)-tol(:);
                obj.cacheRHandPose([s_val s_val],rhand_pose);
                if(brk==1)
                    if(~obj.isBDIManipMode()) % Ignore Foot Constraints in BDIManipMode
                        lfoot_pose.max = lfoot_breaks(:,brk)+tol(:);
                        lfoot_pose.min = lfoot_breaks(:,brk)-tol(:);
                        obj.cacheLFootPose([0 1],lfoot_pose);
                        rfoot_pose.max = rfoot_breaks(:,brk)+tol(:);
                        rfoot_pose.min = rfoot_breaks(:,brk)-tol(:);
                        obj.cacheRFootPose([0 1],rfoot_pose);                  
                    else
                        pelvis_pose= forwardKin(obj.r,kinsol_tmp,obj.pelvis_body,[0;0;0],2);
                        obj.cachePelvisPose([0 1],pelvis_pose);
                    end
                end
                %end
            end
            Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
            Tmax_joints=obj.getTMaxForMaxJointSpeed();
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            send_status(3,0,0,'Cached loaded plan...');
        end
        %-----------------------------------------------------------------------------------------------------------------
        function [varargout]=runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,...
                                        h_ee_constraint,pelvis_constraint,com_constraint,goal_type_flags)
            
            disp('Adjusting plan...');
            send_status(3,0,0,'Adjusting plan...');
            

            q0 = x0(1:getNumDOF(obj.r));
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0);
            r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            r_foot_pts = [0;0;0];
            l_foot_pts = [0;0;0];
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            %======================================================================================================
            if(obj.plan_cache.inTeleopMode)
                q_samples = zeros(obj.r.getNumDOF(),length(obj.plan_cache.s));
                for i =1:length(obj.plan_cache.s)
                    q_samples(:,i) = obj.plan_cache.qtraj.eval(obj.plan_cache.s(i));
                end           
                q_samples(:,1) =  q0; % use current pose
                %============================
                % have to adjust the q at time 0 IK manually
                obj.plan_cache.qtraj = PPTrajectory(spline(obj.plan_cache.s,q_samples));            
            end
            
            
            
            if(isempty(rh_ee_constraint))
                s_int_rh= nan;
                rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
            else
                rhand_int_constraint = zeros(6,1);
                s_int_rh = rh_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of palm in world frame
                rpy = quat2rpy(rh_ee_constraint.desired_pose(4:7));
                T_world_palm_r = HT(rh_ee_constraint.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
                rhand_int_constraint(1:3) = T_world_hand_r(1:3,4);
                rhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                
                if(abs(1-s_int_rh)<1e-3)
                    disp('rh end state is modified')
                    r_hand_poseT(1:3) = rhand_int_constraint(1:3);
                    r_hand_poseT(4:7) = rpy2quat(rhand_int_constraint(4:6));
                    %  replace Boundary Constraint In Cache
                    obj.replaceCachedConstraint(obj.r_hand_body,[1 1],r_hand_poseT(:));
                    rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                    rh_ee_constraint=[];
                end
            end
            
            if(isempty(lh_ee_constraint))
                s_int_lh = nan;
                lhand_int_constraint = [nan;nan;nan;nan;nan;nan];
            else
                lhand_int_constraint = zeros(6,1);
                s_int_lh = lh_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of palm in world frame
                rpy = quat2rpy(lh_ee_constraint.desired_pose(4:7));
                T_world_palm_l = HT(lh_ee_constraint.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
                lhand_int_constraint(1:3) = T_world_hand_l(1:3,4);
                lhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                
                if(abs(1-s_int_lh)<1e-3)
                    disp('lh end state is modified')
                    l_hand_poseT(1:3) = lhand_int_constraint(1:3);
                    l_hand_poseT(4:7) = rpy2quat(lhand_int_constraint(4:6));
                    %  replace Boundary Constraint In Cache
                    obj.replaceCachedConstraint(obj.l_hand_body,[1 1],l_hand_poseT(:));
                    lhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                    lh_ee_constraint=[];
                end
            end
            
            
            
            if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                
                if(isempty(rf_ee_constraint))
                    s_int_rf= nan;
                    rfoot_int_constraint = nan(6,num_r_foot_pts);
                else
                    rfoot_int_constraint = zeros(6,num_r_foot_pts);
                    s_int_rf = rf_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(rf_ee_constraint.desired_pose(4:7,1));
                    for k = 1:num_r_foot_pts
                        T_world_foot_r = HT(rf_ee_constraint.desired_pose(1:3,k),rpy(1),rpy(2),rpy(3));
                        rfoot_int_constraint(1:3,k) = T_world_foot_r(1:3,4);
                        rfoot_int_constraint(4:6,k) =rotmat2rpy(T_world_foot_r(1:3,1:3));
                    end
                    if(abs(1-s_int_rf)<1e-3)
                        disp('rf end state is modified')
                        for k = 1:num_r_foot_pts
                            r_foot_poseT(1:3,k) = rfoot_int_constraint(1:3,k);
                            r_foot_poseT(4:7,k) = rpy2quat(rfoot_int_constraint(4:6,k));
                            %obj.rfootT(:,k) = rfoot_int_constraint(:,k); // Must replace Boundary Constraint In Cache
                        end
                        obj.replaceCachedConstraint(obj.r_foot_body,[1 1],r_foot_poseT);
                        rfoot_int_constraint = nan(6,num_r_foot_pts);
                        rf_ee_constraint=[];
                    end
                end
                
                if(isempty(lf_ee_constraint))
                    s_int_lf= nan;
                    lfoot_int_constraint = nan(6,num_l_foot_pts);
                else
                    lfoot_int_constraint = zeros(6,num_l_foot_pts);
                    s_int_lf = lf_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                    % Desired position of left foot in world frame
                    rpy = quat2rpy(lf_ee_constraint.desired_pose(4:7,1));
                    for k = 1:num_l_foot_pts
                        T_world_foot_l = HT(lf_ee_constraint.desired_pose(1:3,k),rpy(1),rpy(2),rpy(3));
                        lfoot_int_constraint(1:3,k) = T_world_foot_l(1:3,4);
                        lfoot_int_constraint(4:6,k) =rotmat2rpy(T_world_foot_l(1:3,1:3));
                    end
                    if(abs(1-s_int_lf)<1e-3)
                        disp('lf end state is modified')
                        for k = 1:num_l_foot_pts
                            l_foot_poseT(1:3,k) = lfoot_int_constraint(1:3,k);
                            l_foot_poseT(4:7,k) = rpy2quat(lfoot_int_constraint(4:6,k));
                            %obj.lfootT(:,k) = lfoot_int_constraint(:,k); // Must replace Boundary Constraint In Cache
                        end
                        obj.replaceCachedConstraint(obj.l_foot_body,[1 1],l_foot_poseT);
                        lfoot_int_constraint = nan(6,num_l_foot_pts);
                        lf_ee_constraint=[];
                    end
                end
                
            end %end if(~obj.isBDIManipMode())
            
            if(isempty(h_ee_constraint))
                s_int_head= nan;
                head_int_constraint = [nan;nan;nan;nan;nan;nan];
            else
                head_int_constraint = zeros(6,1);
                s_int_head = h_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of palm in world frame
                rpy = quat2rpy(h_ee_constraint.desired_pose(4:7));
                T_world_head = HT(h_ee_constraint.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                head_int_constraint(1:3) = T_world_head(1:3,4);
                head_int_constraint(4:6) =rotmat2rpy(T_world_head(1:3,1:3));
                
                fprintf (1, 'HEAD: Going from POS: (%f,%f,%f) --> (%f,%f,%f) and RPY: (%f,%f,%f) --> (%f,%f,%f)', ...
                    h_ee_constraint.desired_pos(1), ...
                    h_ee_constraint.desired_pos(2),h_ee_constraint.desired_pos(3), ...
                    head_int_constraint(1), ...
                    head_int_constraint(2), ...
                    head_int_constraint(3), rpy(1), rpy(2), ...
                    rpy(3), head_int_constraint(4), ...
                    head_int_constraint(5), head_int_constraint(6));
                
                
                if(abs(1-s_int_head)<1e-3)
                    disp('head end state is modified')
                    head_poseT(1:3) = head_int_constraint(1:3);
                    head_poseT(4:7) = rpy2quat(head_int_constraint(4:6));
                    %obj.headT = head_int_constraint; // Must replace Boundary Constraint In Cache
                    obj.replaceCachedConstraint(obj.head_body,[1 1],head_poseT);
                    head_int_constraint = [nan;nan;nan;nan;nan;nan];
                    h_ee_constraint=[];
                end
            end
            pelvis_pos_tol= 1e-4;
            pelvis_quat_tol=sind(1).^2;
            com_pos_tol= 1e-2;
            if(isempty(pelvis_constraint))
                s_int_pelvis= nan;
                pelvis_int_constraint = [nan;nan;nan;nan;nan;nan];
            else
                pelvis_int_constraint = zeros(6,1);
                s_int_pelvis = pelvis_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of left foot in world frame
                rpy = quat2rpy(pelvis_constraint.desired_pose(4:7));
                T_world_pelvis = HT(pelvis_constraint.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                pelvis_int_constraint(1:3) = T_world_pelvis(1:3,4);
                pelvis_int_constraint(4:6) =rotmat2rpy(T_world_pelvis(1:3,1:3));
                if(abs(1-s_int_pelvis)<1e-3)
                    disp('pelvis end state is modified')
                    pelvis_poseT = zeros(7,1);
                    pelvis_poseT(1:3) = pelvis_int_constraint(1:3);
                    pelvis_poseT(4:7) = rpy2quat(pelvis_int_constraint(4:6));
                    if(~obj.isBDIManipMode())
                        constraint = parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_poseT,pelvis_pos_tol,pelvis_quat_tol,[1 1]);
                        obj.plan_cache.pelvis_constraint_cell = replaceConstraintCell(obj.plan_cache.pelvis_constraint_cell,constraint);
                    else
                        obj.plan_cache.pelvis_constraint_cell = {}; 
                        obj.plan_cache.pelvis_constraint_cell = parse2PosQuatConstraint(obj.r,obj.head_body,[0;0;0],pelvis_poseT,pelvis_pos_tol,pelvis_quat_tol,[0 1]);
                    end
                    pelvis_int_constraint = [nan;nan;nan;nan;nan;nan];
                    pelvis_constraint=[];
                end
            end    
            
            if(isempty(com_constraint))
                s_int_com= nan;
                com_int_constraint = [nan;nan;nan];
            else
                s_int_com = com_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of com in world frame
                com_int_constraint = com_constraint.desired_pose(1:3);
                %com0 = getCOM(obj.r,q0)
                
                if(abs(1-s_int_com)<1e-3)
                    disp('com end state is modified')
                    pos_min = com_int_constraint-com_pos_tol;
                    pos_max = com_int_constraint+com_pos_tol;
                    constraint = {WorldCoMConstraint(obj.r,pos_min,pos_max,[1 1])};
                    obj.plan_cache.com_constraint_cell = replaceConstraintCell(obj.plan_cache.com_constraint_cell,constraint);
                    obj.plan_cache.pelvis_constraint_cell = removeBodyConstraintUtil([1 1],obj.plan_cache.pelvis_constraint_cell); % pelvis constraint can conflict with com constraint
                    com_int_constraint = [nan;nan;nan];
                    com_constraint=[];
                end
            end             
            
            r_hand_pose_int = [rhand_int_constraint(1:3); rpy2quat(rhand_int_constraint(4:6))];
            l_hand_pose_int = [lhand_int_constraint(1:3); rpy2quat(lhand_int_constraint(4:6))];
            if(~obj.isBDIManipMode())
                r_foot_pose_int = [rfoot_int_constraint(1:3,:); repmat(rpy2quat(rfoot_int_constraint(4:6,1)),1,num_r_foot_pts)];
                l_foot_pose_int = [lfoot_int_constraint(1:3,:); repmat(rpy2quat(lfoot_int_constraint(4:6,1)),1,num_l_foot_pts)];
            end
            head_pose_int   = [head_int_constraint(1:3); rpy2quat(head_int_constraint(4:6))];
            pelvis_pose_int = [pelvis_int_constraint(1:3); rpy2quat(pelvis_int_constraint(4:6))];
            com_pose_int = [com_int_constraint(1:3)];
            %======================================================================================================
            
            
            if(~isempty(rh_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_rh)); % snap to closest break point (avoiding very close double constraints)
                s_int_rh=obj.plan_cache.s_breaks(ind);
                rhand_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose_int,1e-2,sind(5).^2,[s_int_rh,s_int_rh]);
                rhand_constraint_cell = obj.plan_cache.rhand_constraint_cell;
                rhand_constraint_cell = removeBodyConstraintUtil([s_int_rh,s_int_rh],rhand_constraint_cell);
                rhand_constraint_cell = [rhand_constraint_cell,rhand_intermediate_constraint];
            else
                rhand_constraint_cell = obj.plan_cache.rhand_constraint_cell;
            end
            %if((~isempty(lh_ee_constraint))&&(abs(1-s_int_lh)>1e-3))
            if(~isempty(lh_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_lh));
                s_int_lh=obj.plan_cache.s_breaks(ind);
                lhand_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose_int,1e-2,sind(5).^2,[s_int_lh,s_int_lh]);
                lhand_constraint_cell = obj.plan_cache.lhand_constraint_cell;
                lhand_constraint_cell = removeBodyConstraintUtil([s_int_lh,s_int_lh],lhand_constraint_cell);
                lhand_constraint_cell = [lhand_constraint_cell,lhand_intermediate_constraint];
            else
                lhand_constraint_cell = obj.plan_cache.lhand_constraint_cell;
            end

            if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                %if((~isempty(rf_ee_constraint))&&(abs(1-s_int_rf)>1e-3))
                if(~isempty(rf_ee_constraint))
                    [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_rf)); % snap to closest break point (avoiding very close double constraints)
                    s_int_rf=obj.plan_cache.s_breaks(ind);
                    tspan = [s_int_rf,s_int_rf];
                    rfoot_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose_int,1e-4,sind(1).^2,tspan);
                    rfoot_constraint_cell = obj.plan_cache.rfoot_constraint_cell;
                    rfoot_constraint_cell = removeBodyConstraintUtil(tspan,rfoot_constraint_cell);
                    rfoot_constraint_cell = [rfoot_constraint_cell,rfoot_intermediate_constraint];
                else
                    rfoot_constraint_cell = obj.plan_cache.rfoot_constraint_cell;
                end
                obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,r_foot_contact_pts);
                
                if(~isempty(lf_ee_constraint))
                    [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_lf));
                    s_int_lf=obj.plan_cache.s_breaks(ind);
                    tspan = [s_int_lf,s_int_lf];
                    lfoot_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose_int,1e-4,sind(1).^2,tspan);
                    lfoot_constraint_cell = obj.plan_cache.lfoot_constraint_cell;
                    lfoot_constraint_cell = removeBodyConstraintUtil(tspan,lfoot_constraint_cell);
                    lfoot_constraint_cell = [lfoot_constraint_cell,lfoot_intermediate_constraint];
                else
                    lfoot_constraint_cell = obj.plan_cache.lfoot_constraint_cell;
                end
                obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.l_foot_body,l_foot_contact_pts);
            else
                if(obj.plan_cache.inTeleopMode)
                    lfoot_constraint_cell = obj.plan_cache.lfoot_constraint_cell;
                    rfoot_constraint_cell = obj.plan_cache.rfoot_constraint_cell;
                else
                    lfoot_constraint_cell = {};
                    rfoot_constraint_cell = {};
                end
                %pelvis_constraint_cell = obj.plan_cache.pelvis_constraint_cell;
            end %end if(~obj.isBDIManipMode())
            
            if(~isempty(h_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_head));
                s_int_head=obj.plan_cache.s_breaks(ind);
                head_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.head_body,[0;0;0],head_pose_int,1e-4,sind(1).^2,[s_int_head,s_int_head]);
                head_constraint_cell = obj.plan_cache.head_constraint_cell;
                head_constraint_cell = removeBodyConstraintUtil([s_int_head,s_int_head],head_constraint_cell);
                head_constraint_cell = [head_constraint_cell,head_intermediate_constraint];
            else
                head_constraint_cell = obj.plan_cache.head_constraint_cell;
            end

            if(~isempty(pelvis_constraint))
                if(~obj.isBDIManipMode())
                    [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_pelvis));
                    s_int_pelvis=obj.plan_cache.s_breaks(ind);
                    pelvis_intermediate_constraint = parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose_int,pelvis_pos_tol,pelvis_quat_tol,[s_int_pelvis,s_int_pelvis]);
                    pelvis_constraint_cell = obj.plan_cache.pelvis_constraint_cell;
                    pelvis_constraint_cell = removeBodyConstraintUtil([s_int_pelvis,s_int_pelvis],pelvis_constraint_cell);
                    pelvis_constraint_cell = [pelvis_constraint_cell,pelvis_intermediate_constraint];
                else
                   lfoot_constraint_cell = {};
                   rfoot_constraint_cell = {}; 
                   obj.plan_cache.pelvis_constraint_cell = {};
                   % NOTE: IkTraj does not adjust starting pose                   
                   obj.plan_cache.pelvis_constraint_cell = parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose_int,pelvis_pos_tol,pelvis_quat_tol,[0 1]);
                   pelvis_constraint_cell = obj.plan_cache.pelvis_constraint_cell;
                end
            else
                pelvis_constraint_cell = obj.plan_cache.pelvis_constraint_cell;
            end 
            if(~isempty(com_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_com));
                s_int_com=obj.plan_cache.s_breaks(ind);
                tspan = [s_int_com,s_int_com];
                com_intermediate_constraint = {WorldCoMConstraint(obj.r,com_pose_int-com_pos_tol,com_pose_int+com_pos_tol,tspan)};
                pelvis_constraint_cell = removeBodyConstraintUtil(tspan,pelvis_constraint_cell); % pelvis constraint can conflict with com constraint
                com_constraint_cell = obj.plan_cache.com_constraint_cell;
                com_constraint_cell = removeBodyConstraintUtil([s_int_com,s_int_com],com_constraint_cell);
                com_constraint_cell = [com_constraint_cell,com_intermediate_constraint];
            else
                com_constraint_cell = obj.plan_cache.com_constraint_cell;
            end               
           
            
            
            %iktraj_tbreaks = linspace(0,1,obj.plan_cache.num_breaks);
            iktraj_tbreaks = [0 1];
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,lhand_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,rhand_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,lfoot_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,rfoot_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,head_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,pelvis_constraint_cell);
            iktraj_tbreaks = addIKtrajTimeBreaks(iktraj_tbreaks,com_constraint_cell);
            % PERFORM IKSEQUENCE OPT
            cost = getCostVector(obj);
            iktraj_options = IKoptions(obj.r);
            iktraj_options = iktraj_options.setDebug(true);
            iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
            iktraj_options = iktraj_options.setQa(0.05*eye(getNumDOF(obj.r)));
            iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
            iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1));            
            iktraj_options = iktraj_options.setMajorIterationsLimit(400);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            q_seed = obj.plan_cache.qtraj.eval(iktraj_tbreaks(2:end));
            q_seed_traj = PPTrajectory(foh(iktraj_tbreaks,[q0 q_seed]));
            q0 = obj.plan_cache.qtraj.eval(0); % use start of cached trajectory instead of current
            
            %if((~isempty(pelvis_constraint))&&(obj.isBDIManipMode()))
            %   q0(1:3) =  pelvis_pose_int(1:3);
            %   q0(4:6) =  quat2rpy(pelvis_pose_int(4:7));
            %end
            
            
            q_nom = obj.plan_cache.qtraj.eval(iktraj_tbreaks(2:end));
            q_nom_traj = PPTrajectory(foh(iktraj_tbreaks,[q0 q_nom]));
            
            %============================
            %if(length(iktraj_tbreaks)<=5)
            if(~obj.plan_cache.isPointWiseIK)
                obj.plan_cache.qsc = obj.plan_cache.qsc.setShrinkFactor(0.85);
                

                [xtraj,snopt_info,infeasible_constraint] = inverseKinTrajWcollision(obj.r,obj.collision_check,...
                    iktraj_tbreaks,q_seed_traj,q_nom_traj,...
                    lhand_constraint_cell{:},rhand_constraint_cell{:},...
                    lfoot_constraint_cell{:},rfoot_constraint_cell{:},...
                    pelvis_constraint_cell{:},com_constraint_cell{:},head_constraint_cell{:},...
                    obj.joint_constraint,obj.plan_cache.qsc,iktraj_options);
                xtraj = xtraj.setOutputFrame(obj.r.getStateFrame());
                x_breaks = xtraj.eval(iktraj_tbreaks);
                
                if(snopt_info > 10)
                    warning('The IK traj fails');
                    send_msg = sprintf('snopt_info == %d. The IKtraj fails.',snopt_info);
                    send_status(4,0,0,send_msg);
                    display(infeasibleConstraintMsg(infeasible_constraint));
                end
                
            else
                iktraj_options = iktraj_options.setSequentialSeedFlag(true);
                [xtraj,snopt_info,infeasible_constraint] = inverseKinPointwise(obj.r,...
                    iktraj_tbreaks,q_seed,q_nom,...
                    lhand_constraint_cell{:},rhand_constraint_cell{:},...
                    lfoot_constraint_cell{:},rfoot_constraint_cell{:},...
                    pelvis_constraint_cell{:},com_constraint_cell{:},head_constraint_cell{:},...
                    obj.joint_constraint,obj.plan_cache.qsc,iktraj_options);
                x_breaks = xtraj;
                %snopt_info
                display(infeasibleConstraintMsg(infeasible_constraint));
                for k=1:length(snopt_info),
                    if(snopt_info(k) > 10)
                        warning('The IK fails');
                        send_msg = sprintf('snopt_info == %d. The IKtraj fails at %d.',snopt_info(k),k);
                        send_status(4,0,0,send_msg);
                    end
                end
                
            end
            
            
            %============================
            
            s_breaks = iktraj_tbreaks;
            q_breaks = x_breaks(1:obj.r.getNumDOF,:);
            %s_total = Tmax_ee*obj.plan_cache.v_desired;
            %s = linspace(0,1,max(ceil(s_total/obj.plan_arc_res)+1,15)); % Must have two points atleast
            
            grasp_transition_breaks = obj.plan_cache.grasp_transition_breaks;
            s = obj.plan_cache.s;
            obj.plan_cache.s_breaks = linspace(0,1,obj.plan_cache.num_breaks);
            s = unique([s(:);obj.plan_cache.s_breaks(:);grasp_transition_breaks(:)]);
            obj.plan_cache.s = s;
            qtraj_guess = PPTrajectory(spline(iktraj_tbreaks,q_breaks));
            % fine grained sampling of plan.
            q = zeros(obj.r.getNumDOF,length(s));
            q(:,1) = q_breaks(:,1);
            for i=2:length(s)
                si = s(i);
                q(:,i) =qtraj_guess.eval(si);
            end
            
            % update cache (will be overwritten when setPlanCache is called)
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            obj.plan_cache.lhand_constraint_cell = lhand_constraint_cell;
            obj.plan_cache.rhand_constraint_cell = rhand_constraint_cell;
            obj.plan_cache.lfoot_constraint_cell = lfoot_constraint_cell;
            obj.plan_cache.rfoot_constraint_cell = rfoot_constraint_cell;
            obj.plan_cache.pelvis_constraint_cell = pelvis_constraint_cell;
            obj.plan_cache.head_constraint_cell = head_constraint_cell;
            
            % publish plan
            if nargout == 0
              disp('Publishing plan...');
            end
            nx_atlas = length(obj.atlas2robotFrameIndMap);
            xtraj_atlas = zeros(nx_atlas+2,length(s));
            xtraj_atlas(1,:) = 0*s;
            xtraj_atlas(2,:) = 0*s;
            
            for l = 1:length(obj.plan_cache.s_breaks),
                xtraj_atlas(1,s == obj.plan_cache.s_breaks(l)) = 1.0;
            end
            
            % Set the breakpoints here if they exist.
            for l = 1:length(grasp_transition_breaks),
                xtraj_atlas(2,s == grasp_transition_breaks(l)) = 1.0;
            end
            
            xtraj_atlas(2+(1:nx_atlas/2),:) = q(obj.atlas2robotFrameIndMap(1:nx_atlas/2),:);
            
            Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
            Tmax_joints=obj.getTMaxForMaxJointSpeed();
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            old_time_2_index_scale =  obj.plan_cache.time_2_index_scale;
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            
            snopt_info_vector = max(snopt_info)*ones(1, size(xtraj_atlas,2));
            utime = now() * 24 * 60 * 60;
            if(~obj.plan_cache.isEndPose)
                if(obj.plan_cache.num_grasp_transitions>0)
                    G = obj.plan_cache.grasp_transition_states;
                    % must readjust G(i).utime according to grasp_transition_breaks
                    if(length([G.utime])>1)
                        for k=1:length([G.utime]),
                            [~,ind]=min(abs(grasp_transition_breaks-G(k).utime*(old_time_2_index_scale)));
                            G(k).utime = grasp_transition_breaks(ind).*(1/obj.plan_cache.time_2_index_scale);
                        end
                    end
                    if(nargout==0)
                      send_status(3,0,0,'Published Adjusted plan...');
                      obj.plan_pub.publish(xtraj_atlas,ts,utime,snopt_info_vector,G);
                    else
                      varargout{1}=xtraj_atlas;  varargout{2}=ts;  
                      varargout{3}=snopt_info_vector;
                      varargout{4}= G;
                    end
                else
                   if(nargout==0)
                      send_status(3,0,0,'Published Adjusted plan...');
                      obj.plan_pub.publish(xtraj_atlas,ts,utime);
                   else
                      varargout{1}=xtraj_atlas;  varargout{2}=ts;  
                   end

                end
               
            else
                xtraj_atlas = zeros(nx_atlas,1);
                q1 = obj.plan_cache.qtraj.eval(1);
                xtraj_atlas(1:nx_atlas/2) = q1(obj.atlas2robotFrameIndMap); % only publish the last state as an EndPose
                obj.pose_pub.publish(xtraj_atlas,utime);
            end
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
    end% end methods
end% end classdef


% -------------------- GRAVEYARD -------------
%-----------------------------------------------------------------------------------------------------------------
%function adjustCachedPlanToCurrentRobotState(obj,x0,mode)
%   % append posture plan to the cached plan;
%  if(obj.plan_cache.num_grasp_transitions>0)
%      [xtraj,ts,snopt_info_vector,G] = adjustCachedPlanToCurrentPelvisPose(obj,x0,mode);
%  else
%      [xtraj,ts] = adjustCachedPlanToCurrentPelvisPose(obj,x0,mode);
%  end        

%  % Decide if a posture plan needs to prepended if the current robot state is not the same as the first frame in the cached plan.
%  q0 = x0(1:getNumDOF(obj.r));
%  s = [0 1];
%  nq = obj.r.getNumDOF();
%  q_start = obj.plan_cache.qtraj.eval(0);
%  
%  s_breaks = linspace(0,1,4);
%  cost = getCostVector(obj);
%  Q = diag(cost(1:getNumDOF(obj.r)));
%  cost = (q0-q_start)'* Q*(q0-q_start); 
%  cost_TOL = 1e-4;
%  if(cost>cost_TOL)
%    q_breaks = zeros(nq,length(s_breaks));
%    qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_start]));  
%    for brk =1:length(s_breaks),
%      q_breaks(:,brk) = qtraj_guess.eval(s_breaks(brk));
%    end 
%    Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
%    s_total = Tmax_ee*obj.plan_cache.v_desired;
%    s= linspace(0,1,max(ceil(s_total/obj.plan_arc_res)+1,5)); % Must have two points atleast
%    s = unique([s(:);s_breaks(:)]);
%    q = zeros(length(q0),length(s));
%    for i=1:length(s),
%        q(:,i) = qtraj_guess.eval(s(i));
%    end            
%    xtraj_prepend = zeros(getNumStates(obj.r)+2,length(s));
%    xtraj_prepend(1,:) = 0*s;
%    for l =1:length(s_breaks),
%        ind = find(abs(s - s_breaks(l))<1e-3);
%        xtraj_prepend(1,ind) = 1.0;
%        xtraj_prepend(2,ind) = 0.0;
%    end
%    xtraj_prepend(2+(1:nq),:) = q;          
%    Tmax_joints=obj.getTMaxForMaxJointSpeed();
%    ts_prepend = s.*max(Tmax_joints,Tmax_ee); % plan timesteps   
%    
%    % for debug
%    utime = now() * 24 * 60 * 60;              
%    obj.plan_pub.publish(xtraj_prepend,ts_prepend,utime);
%  end
%  
% 
%  
%  
% % prepend
% % this will be difficult for retractable plans
%      xtraj=[xtraj_prepend xtraj];
%      dt=ts_prepend(2)-ts_prepend(1);
%      t_offset = ts_prepend(end) +dt;
%      ts=[ts_prepend;ts+ts_prepend(end)+dt];
%      if(obj.plan_cache.num_grasp_transitions>0)
%         % must readjust G(i).utime according to grasp_transition_breaks
%         if(length([G.utime])>1)
%             for k=1:length([G.utime]),
%                 G(k).utime = G(k).utime + t_offset;
%              end
%          end
%      end
%      % update cache and publish
%      pause(0.1);
%      utime = now() * 24 * 60 * 60; 
%      obj.plan_pub.publish(xtraj,ts,utime);

%end    
