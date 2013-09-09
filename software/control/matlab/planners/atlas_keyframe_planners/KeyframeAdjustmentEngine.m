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
        cached_plan_s
    end
    
    methods
        function obj = KeyframeAdjustmentEngine(r,hardware_mode)            
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names); % if endpose flag is set
            obj.plan_cache = KeyframePlanCache();
        end
        
        function setPlanCache(obj,cache)
            obj.plan_cache  = cache;
        end
        
        function adjustAndPublishCachedPlan(obj,x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint,goal_type_flags)
            runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,goal_type_flags);
        end
        
        function adjustCachedPlanToCurrentPelvisPose(obj,x0,mode)
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
            
            for i =1:length(obj.cached_plan_s),
                q_samples(:,i) = obj.plan_cache.qtraj.eval(obj.cached_plan_s(i));
            end

            constraints = {};
            kinsol_start = doKinematics(obj.r,q_samples(:,1));
            if(mode==drc.plan_adjust_mode_t.LEFT_HAND)
              obj.removeConstraintsContainingStr('lfoot');
              obj.removeConstraintsContainingStr('rfoot');
              obj.removeConstraintsContainingStr('rhand');
              lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
              constraints = {obj.l_hand_body,[0;0;0],lhand_pose};
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_HAND)
              obj.removeConstraintsContainingStr('lfoot');
              obj.removeConstraintsContainingStr('rfoot');
              obj.removeConstraintsContainingStr('lhand');
              rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
              constraints = {obj.r_hand_body,[0;0;0],rhand_pose};
            elseif(mode==drc.plan_adjust_mode_t.BOTH_HANDS)
              obj.removeConstraintsContainingStr('lfoot');
              obj.removeConstraintsContainingStr('rfoot');
              lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
              rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
              constraints = {obj.l_hand_body,[0;0;0],lhand_pose,obj.r_hand_body,[0;0;0],rhand_pose};
            elseif(mode==drc.plan_adjust_mode_t.LEFT_FOOT)
              obj.removeConstraintsContainingStr('lhand');
              obj.removeConstraintsContainingStr('rhand');
              obj.removeConstraintsContainingStr('rfoot');
              lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
              constraints = {obj.l_foot_body,[0;0;0],lfoot_pose};
            elseif(mode==drc.plan_adjust_mode_t.RIGHT_FOOT)
              obj.removeConstraintsContainingStr('lhand');
              obj.removeConstraintsContainingStr('rhand');
              obj.removeConstraintsContainingStr('lfoot');
              rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
              constraints = {obj.r_foot_body,[0;0;0],rfoot_pose};
            elseif(mode==drc.plan_adjust_mode_t.BOTH_FEET)
              obj.removeConstraintsContainingStr('lhand');
              obj.removeConstraintsContainingStr('rhand');
              lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
              rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
              constraints = {obj.l_foot_body,[0;0;0],lfoot_pose,obj.r_foot_body,[0;0;0],rfoot_pose};
            elseif(mode==drc.plan_adjust_mode_t.ALL)
              % dont remove anything
              lhand_pose = forwardKin(obj.r,kinsol_start,obj.l_hand_body,[0;0;0],2);
              rhand_pose = forwardKin(obj.r,kinsol_start,obj.r_hand_body,[0;0;0],2);
              lfoot_pose = forwardKin(obj.r,kinsol_start,obj.l_foot_body,[0;0;0],2);
              rfoot_pose = forwardKin(obj.r,kinsol_start,obj.r_foot_body,[0;0;0],2);
              constraints = {obj.l_hand_body,[0;0;0],lhand_pose,obj.r_hand_body,[0;0;0],rhand_pose,...
                            obj.l_foot_body,[0;0;0],lfoot_pose,obj.r_foot_body,[0;0;0],rfoot_pose};
            end
            
            %============================
            cost = getCostVector(obj);
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.quasiStaticFlag = false;
            ikoptions.shrinkFactor = 0.85;
            ikoptions.MajorIterationsLimit = 500;
            ikoptions.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
            ikoptions.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
            constraints = [{obj.pelvis_body,[0;0;0],pelvis_pose}, constraints];
            [q_first,snopt_info] = inverseKin(obj.r,q0,constraints{:},ikoptions);
            if(snopt_info > 10)
                warning('The IK sequence fails');
                send_status(4,0,0,sprintf('snopt_info == %d.  IK failed to adjust first pose.',snopt_info));
            end
            %============================
            q_samples(:,1) =  q_first;
                       
            % have to adjust the q at time 0 with a IK manually to the desired pelvis pose.
            % Ik Sequence by design does not modify the first posture at time zero.
            obj.plan_cache.qtraj = PPTrajectory(spline(obj.cached_plan_s,q_samples));
            
             % delete old and add new
            if(sum(strcmp(obj.plan_cache.ks.kincon_name,'pelvis'))>0) %exists
                obj.plan_cache.ks = deleteKinematicConstraint(obj.plan_cache.ks,'pelvis');
            end
            obj.cachePelvisPose([0 1],'pelvis',pelvis_pose);
            runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,goal_type_flags);
        end
        
        function setCacheViaPlanMsg(obj,xtraj,ts,grasptransitions,logictraj)
            
            obj.plan_cache.isEndPose = false;
            obj.plan_cache.num_breaks = sum(logictraj(1,:));
            obj.plan_cache.ks = ActionSequence(); % Plan Boundary Conditions
            s_breaks = linspace(0,1,length(find(logictraj(1,:)==1)));%ts(find(logictraj(1,:)==1));
            obj.plan_cache.s_breaks = s_breaks;
     
            s = linspace(0,1,length(ts));

            obj.plan_cache.qtraj = PPTrajectory(spline(s,xtraj(1:getNumDOF(obj.r),:)));
            obj.plan_cache.quasiStaticFlag = false;
            s_breaks = s;
            obj.cached_plan_s = s;
            
            nq = obj.r.getNumDOF();
            q_break = zeros(nq,length(s_breaks));
            rhand_breaks = zeros(7,length(s_breaks));
            lhand_breaks = zeros(7,length(s_breaks));
            head_breaks = zeros(7,length(s_breaks));
            rfoot_breaks = zeros(7,length(s_breaks));
            lfoot_breaks = zeros(7,length(s_breaks));
            for brk =1:length(s_breaks),
                q_break(:,brk) = obj.plan_cache.qtraj.eval(s_breaks(brk));
                kinsol_tmp = doKinematics(obj.r,q_break(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
                %if((brk==1)||brk==length(s_breaks))
                    Tag =    num2str(brk-1);
                    if(brk==length(s_breaks))
                        Tag ='T';
                    end
                    s_val = s_breaks(brk);                    
                    obj.cacheLHandPose([s_val s_val],['lhand' Tag],lhand_breaks(:,brk));                 
                    obj.cacheRHandPose([s_val s_val],['rhand' Tag],rhand_breaks(:,brk));
                    if(brk==1)
                        if(~obj.isBDIManipMode()) % Ignore Foot Constraints in BDIManipMode
                            obj.cacheLFootPoseAsContactConstraint([0 1],['lfoot' Tag],lfoot_breaks(:,brk));
                            obj.cacheRFootPoseAsContactConstraint([0 1],['rfoot' Tag],rfoot_breaks(:,brk));
                            pelvis_pose= forwardKin(obj.r,kinsol_tmp,obj.pelvis_body,[0;0;0],2);
                            obj.cachePelvisPose([0 1],'pelvis',pelvis_pose);
                        else
                            pelvis_pose= forwardKin(obj.r,kinsol_tmp,obj.pelvis_body,[0;0;0],2);
                            obj.cachePelvisPose([0 1],'pelvis',pelvis_pose);
                        end
                    end
               %end
            end
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head);
%           s_total = max(max(s_total_lh,s_total_rh),s_total_head);
            s_total = max(s_total,0.01);
            
            dqtraj=fnder(obj.plan_cache.qtraj,1); 
            sfine = linspace(s(1),s(end),50);
            Tmax_joints = max(max(abs(eval(dqtraj,sfine)),[],2))/obj.plan_cache.qdot_desired;
            Tmax_ee  = (s_total/obj.plan_cache.v_desired);
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));  

                    
        end
        
        
        function runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,goal_type_flags)
            
            disp('Adjusting plan...');
            send_status(3,0,0,'Adjusting plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0);
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);

            
            %======================================================================================================
            
            
            if(isempty(rh_ee_constraint))
                s_int_rh= nan;
                rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
            else
                rhand_int_constraint = zeros(6,1);
                s_int_rh = rh_ee_constraint.time*obj.plan_cache.time_2_index_scale;
                % Desired position of palm in world frame
                rpy = quat2rpy(rh_ee_constraint.desired_pose(4:7));
                T_world_palm_r = HT(rh_ee_constraint.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r_sandia;
                rhand_int_constraint(1:3) = T_world_hand_r(1:3,4);
                rhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                
                if(abs(1-s_int_rh)<1e-3)
                    disp('rh end state is modified')
                    r_hand_poseT(1:3) = rhand_int_constraint(1:3);
                    r_hand_poseT(4:7) = rpy2quat(rhand_int_constraint(4:6));
                    obj.replaceCachedConstraint('rhandT',r_hand_poseT(:));
                    %obj.rhandT = rhand_int_constraint; // Must replace Boundary Constraint In Cache
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
                T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l_sandia;
                lhand_int_constraint(1:3) = T_world_hand_l(1:3,4);
                lhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                
                if(abs(1-s_int_lh)<1e-3)
                    disp('lh end state is modified')
                    l_hand_poseT(1:3) = lhand_int_constraint(1:3);
                    l_hand_poseT(4:7) = rpy2quat(lhand_int_constraint(4:6));
                    obj.replaceCachedConstraint('lhandT',l_hand_poseT(:));
                    %obj.lhandT = lhand_int_constraint; // Must replace Boundary Constraint In Cache
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
                    for k = 1:num_r_foot_ptsobj.plan_cache.s_breaks
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
                        obj.replaceCachedConstraint('rfootT',r_foot_poseT);
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
                        obj.replaceCachedConstraint('lfootT',l_foot_poseT);
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
                    obj.replaceCachedConstraint('headT',head_poseT);
                    head_int_constraint = [nan;nan;nan;nan;nan;nan];
                    h_ee_constraint=[];
                end
            end
            
            r_hand_pose_int = [rhand_int_constraint(1:3); rpy2quat(rhand_int_constraint(4:6))];
            l_hand_pose_int = [lhand_int_constraint(1:3); rpy2quat(lhand_int_constraint(4:6))];
            if(~obj.isBDIManipMode())
                r_foot_pose_int = [rfoot_int_constraint(1:3,:); repmat(rpy2quat(rfoot_int_constraint(4:6,1)),1,num_r_foot_pts)];
                l_foot_pose_int = [lfoot_int_constraint(1:3,:); repmat(rpy2quat(lfoot_int_constraint(4:6,1)),1,num_l_foot_pts)];
            end
            head_pose_int   = [head_int_constraint(1:3); rpy2quat(head_int_constraint(4:6))];
            
            
            %======================================================================================================
            
            % Load cached actionSequence (boundary constraints of cached plan)
            ks = obj.plan_cache.ks;% ActionSequence();
            
            % If break point is adjusted via gui.
            r_hand_pose_int_relaxed.min= r_hand_pose_int-1e-3;
            r_hand_pose_int_relaxed.max= r_hand_pose_int+1e-3;
            l_hand_pose_int_relaxed.min= l_hand_pose_int-1e-3;
            l_hand_pose_int_relaxed.max= l_hand_pose_int+1e-3;
            if(~obj.isBDIManipMode())
                r_foot_pose_int_relaxed.min= r_foot_pose_int-1e-3;
                r_foot_pose_int_relaxed.max= r_foot_pose_int+1e-3;
                l_foot_pose_int_relaxed.min= l_foot_pose_int-1e-3;
                l_foot_pose_int_relaxed.max= l_foot_pose_int+1e-3;
            end
            head_pose_int_relaxed.min= head_pose_int-1e-3;
            head_pose_int_relaxed.max= head_pose_int+1e-3;
            
            %if((~isempty(rh_ee_constraint))&&(abs(1-s_int_rh)>1e-3))
            if(~isempty(rh_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_rh)); % snap to closest break point (avoiding very close double constraints)
                s_int_rh=obj.plan_cache.s_breaks(ind);
                kc_rhand_intermediate = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose_int,[s_int_rh,s_int_rh],'rhand_int');
                ks = ks.addKinematicConstraint(kc_rhand_intermediate);
            end
            %if((~isempty(lh_ee_constraint))&&(abs(1-s_int_lh)>1e-3))
            if(~isempty(lh_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_lh));
                s_int_lh=obj.plan_cache.s_breaks(ind);
                kc_lhand_intermediate = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose_int,[s_int_lh,s_int_lh],'lhand_int');
                ks = ks.addKinematicConstraint(kc_lhand_intermediate);
            end
            
            
            if(~obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode
                %if((~isempty(rf_ee_constraint))&&(abs(1-s_int_rf)>1e-3))
                if(~isempty(rf_ee_constraint))
                    [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_rf)); % snap to closest break point (avoiding very close double constraints)
                    s_int_rf=obj.plan_cache.s_breaks(ind);
                    kc_rfoot_intermediate = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose_int,[s_int_rf,s_int_rf],'rfoot_int',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_rfoot_intermediate);
                end
                %if((~isempty(lf_ee_constraint))&&(abs(1-s_int_lf)>1e-3))
                if(~isempty(lf_ee_constraint))
                    [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_lf));
                    s_int_lf=obj.plan_cache.s_breaks(ind);
                    kc_lfoot_intermediate = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose_int,[s_int_lf,s_int_lf],'lfoot_int',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_lfoot_intermediate);
                end
            end %end if(~obj.isBDIManipMode())
            
            if(~isempty(h_ee_constraint))
                [~,ind] = min(abs(obj.plan_cache.s_breaks-s_int_head));
                s_int_head=obj.plan_cache.s_breaks(ind);
                kc_head_intermediate = ActionKinematicConstraint(obj.r,obj.head_body,[0;0;0],head_pose_int,[s_int_head,s_int_head],'head_int');
                %                     ks = ks.addKinematicConstraint(kc_head_intermediate);
            end
            
            % TODO: CoM Keyframe Adjustment
            % compute fixed COM goal
            %  gc = contactPositions(obj.r,q0);
            %  k = convhull(gc(1:2,:)');
            %  com0 = getCOM(obj.r,q0);
            %  comgoal = [mean(gc(1:2,k),2);com0(3)];
            %  comgoal = com0; % DOnt move com for now as this is pinned manipulation
            %  comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            %  comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            % kc_com = ActionKinematicConstraint(obj.r,0,[0;0;0],comgoal,[s(1),s(end)],'com');
            % ks = ks.addKinematicConstraint(kc_com);
            
            % PERFORM IKSEQUENCE OPT
            cost = getCostVector(obj);
            ikseq_options.Q = diag(cost(1:getNumDOF(obj.r)));
            ikseq_options.Qa = eye(getNumDOF(obj.r));
            ikseq_options.Qv = eye(getNumDOF(obj.r));
            ikseq_options.nSample = obj.plan_cache.num_breaks-1;
            ikseq_options.qdotf.lb = zeros(obj.r.getNumDOF(),1);
            ikseq_options.qdotf.ub = zeros(obj.r.getNumDOF(),1);
            if(obj.isBDIManipMode()) % Ignore Feet In BDI Manip Mode     
                ikseq_options.Qa = 0.05*eye(getNumDOF(obj.r));
                ikseq_options.Qv = 0*eye(getNumDOF(obj.r));       
            end
            ikseq_options.quasiStaticFlag=obj.plan_cache.quasiStaticFlag;
            ikseq_options.shrinkFactor = 0.9;
            ikseq_options.MajorIterationsLimit = 200; 
            ikseq_options.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());
            ikseq_options.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            ikseq_options.qtraj0 = obj.plan_cache.qtraj; % use previous optimization output as seed
            q0 = obj.plan_cache.qtraj.eval(0); % use start of cached trajectory instead of current
            ikseq_options.q_traj_nom = ikseq_options.qtraj0; % Without this the cost function is never used
            %============================
            [s_breaks,q_breaks,qdos_breaks,qddos_breaks,snopt_info] = inverseKinSequence(obj.r,q0,0*q0,ks,ikseq_options);
            if(snopt_info > 10)
                warning('The IK sequence fails');
                send_status(4,0,0,sprintf('snopt_info == %d. The IK sequence fails.',snopt_info));
            end
            %============================
            xtraj = PPTrajectory(pchipDeriv(s_breaks,[q_breaks;qdos_breaks],[qdos_breaks;qddos_breaks]));
            xtraj = xtraj.setOutputFrame(obj.r.getStateFrame());
            
            obj.plan_cache.s_breaks = s_breaks;
            qtraj_guess = PPTrajectory(spline(s_breaks,q_breaks));
            
            
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

            s= linspace(0,1,max(ceil(s_total/res)+1,15)); % Must have two points atleastks
            s = unique([s(:);s_breaks(:)]);
            % fine grained sampling of plan.
            for i=2:length(s)
                si = s(i);
                q(:,i) =qtraj_guess.eval(si);
            end
            
            % update cache (will be overwritten when setPlanCache is called)
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            
            % publish plan
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
            
            
            dqtraj=fnder(obj.plan_cache.qtraj,1); 
            sfine = linspace(s(1),s(end),50);
            Tmax_joints = max(max(abs(eval(dqtraj,sfine)),[],2))/obj.plan_cache.qdot_desired;
            Tmax_ee  = (s_total/obj.plan_cache.v_desired);
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            
            utime = now() * 24 * 60 * 60;
            if(~obj.plan_cache.isEndPose)
                obj.plan_pub.publish(xtraj,ts,utime);
            else
                xtraj = zeros(getNumStates(obj.r),1);
                xtraj(1:getNumDOF(obj.r),:) = obj.plan_cache.qtraj.eval(1); % only publish the last state as an EndPose
                obj.pose_pub.publish(xtraj,utime);
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
