classdef ManipulationPlanner < handle
    
    properties
        num_breaks
        s_breaks
        q_breaks
        qdos_breaks
        plan_pub
        map_pub
        r
        lhandT % cache goals
        rhandT
        lfootT % cache goals
        rfootT
        headT
        qtraj_guess % old coarse keyframe sequence search
        qtraj_guess_fine % fine manip plan with individual IK fill ins
        time_2_index_scale
        restrict_feet
    end
    
    methods
        function obj = ManipulationPlanner(r)
            obj.r = r;
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.num_breaks = 4;
            obj.plan_pub = RobotPlanPublisherWKeyFrames('atlas',joint_names,true,'CANDIDATE_MANIP_PLAN',obj.num_breaks);
            obj.map_pub = AffIndexedRobotPlanPublisher('CANDIDATE_MANIP_MAP',true,joint_names);
            restrict_feet=true;
        end
        
        function adjustAndPublishManipulationPlan(obj,x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint)
            is_keyframe_constraint = true;
            runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,is_keyframe_constraint);
        end
        
        function generateAndPublishManipulationPlan(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal)
            is_keyframe_constraint = false;
            %rf_ee_goal=[];lf_ee_goal=[];
            runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,is_keyframe_constraint);
            %runOptimization_old(obj,x0,rh_ee_goal,lh_ee_goal,is_keyframe_constraint);
        end
        
        function generateAndPublishManipulationMap(obj,x0,ee_names,ee_loci,affIndices)
            runOptimizationForManipMapGivenEELoci(obj,x0,ee_names,ee_loci,affIndices);
        end

        function runOptimizationForManipMapGivenEELoci(obj,x0,ee_names,ee_loci,affIndices)
            
            disp('Generating manip map...');
            
            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
            rfoot_body = obj.r.findLink('r_foot');
            lfoot_body = obj.r.findLink('l_foot');
            head_body = obj.r.findLink('head');
            
            % r_foot_pose0 = forwardKin(obj.r,kinsol,rfoot_body,...
            %    rfoot_body.contact_pts(:,[rfoot_body.collision_group{1}]),2);
            % r_foot_pose0 = mean(r_foot_pose0,2);
            % l_foot_pose0 = forwardKin(obj.r,kinsol,lfoot_body,...
            %    lfoot_body.contact_pts(:,[lfoot_body.collision_group{1}]),2);
            % l_foot_pose0 = mean(l_foot_pose0,2);
            r_foot_pose0 = forwardKin(obj.r,kinsol,rfoot_body,[0;0;0],2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,lfoot_body,[0;0;0],2);
            head_pose0 = forwardKin(obj.r,kinsol,head_body,[0;0;0],2);
            head_pose0_relaxed.min=head_pose0-[1e-2*ones(3,1);1e-2*ones(4,1)];
            head_pose0_relaxed.max=head_pose0+[1e-2*ones(3,1);1e-2*ones(4,1)];
            % compute fixed COM goal
            gc = contactPositions(obj.r,q0);
            k = convhull(gc(1:2,:)');
            com0 = getCOM(obj.r,q0);
            %   comgoal = [mean(gc(1:2,k),2);com0(3)];
            %   comgoal = com0; % DOnt move com for now as this is pinned manipulation
            comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            
            r_hand_body = findLink(obj.r,'r_hand');
            l_hand_body = findLink(obj.r,'l_hand');
            
            % compute EE trajectories
            r_hand_pose0 = forwardKin(obj.r,kinsol,r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,l_hand_body,[0;0;0],2);
            
            pelvis_body = findLink(obj.r,'pelvis'); % dont move pelvis
            pelvis_pose0 = forwardKin(obj.r,kinsol,pelvis_body,[0;0;0],2);
            utorso_body = findLink(obj.r,'utorso'); % dont move pelvis
            utorso_pose0 = forwardKin(obj.r,kinsol,utorso_body,[0;0;0],2);
            utorso_pose0_relaxed = utorso_pose0;
            utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-2*ones(4,1)];
            utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-2*ones(4,1)];
            % utorso_pose0 = utorso_pose0(1:3);
            
            
            
            % Hand Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            T_hand_palm_l = HT([0;0.1;0],1.57079,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],-1.57079,0,-1.57079);
            T_palm_hand_r = inv_HT(T_hand_palm_r);
            
            
            ind = getActuatedJoints(obj.r);
            cost = getCostVector2(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.MajorIterationsLimit = 100;
            
            % Solve IK for each element i n EE LOCII
            affIndicesTimes = unique([affIndices.time]);
            N = length(affIndicesTimes);
            %plan_affIndices= javaArray('drc.affordance_index_t', N);
            plan_affIndices=[];
            q_guess =q0;
            for i=1:length(affIndicesTimes),
                tic;
                
                %l_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                lhand_const.min = l_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                lhand_const.max = l_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                %r_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                rhand_const.min = r_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                rhand_const.max = r_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                %l_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
                lfoot_const.min = l_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                lfoot_const.max = l_foot_pose0+1e-2*[ones(3,1);ones(4,1)];
                %r_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
                rfoot_const.min = r_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                rfoot_const.max = r_foot_pose0+1e-2*[ones(3,1);ones(4,1)];
                
                plan_affIndices(i).time=affIndices(i).time;
                plan_affIndices(i).aff_type=affIndices(i).aff_type;
                plan_affIndices(i).aff_uid=affIndices(i).aff_uid;
                plan_affIndices(i).num_ees=0;
                plan_affIndices(i).ee_name=[];
                plan_affIndices(i).dof_name=[];
                plan_affIndices(i).dof_value=[];
                ind=find([affIndices.time]==affIndicesTimes(i));
                % Find all active constraints at current index
                for k=1:length(ind),
                    
                    if(strcmp('left_palm',ee_names{ind(k)}))
                        l_ee_goal = ee_loci(:,ind(k));
                        lhandT = zeros(6,1);
                        T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                        T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                        lhandT(1:3) = T_world_hand_l(1:3,4);
                        lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                        l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                        lhand_const.min = l_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                        lhand_const.max = l_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                        plan_affIndices(i).aff_type=affIndices(ind(k)).aff_type;
                        plan_affIndices(i).aff_uid=affIndices(ind(k)).aff_uid;
                        plan_affIndices(i).num_ees=plan_affIndices(i).num_ees+1;
                        plan_affIndices(i).ee_name=[plan_affIndices(i).ee_name;java.lang.String('l_hand')];
                        plan_affIndices(i).dof_name=[plan_affIndices(i).dof_name;affIndices(ind(k)).dof_name(1)];
                        plan_affIndices(i).dof_value=[plan_affIndices(i).dof_value;affIndices(ind(k)).dof_value(1)];
                    end
                    
                    if(strcmp('right_palm',ee_names{ind(k)}))
                        r_ee_goal = ee_loci(:,ind(k));
                        rhandT = zeros(6,1);
                        T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                        T_world_hand_r = T_world_palm_r*T_palm_hand_r;
                        rhandT(1:3) = T_world_hand_r(1:3,4);
                        rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                        r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                        rhand_const.min = r_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                        rhand_const.max = r_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                        plan_affIndices(i).aff_type=affIndices(ind(k)).aff_type;
                        plan_affIndices(i).aff_uid=affIndices(ind(k)).aff_uid;
                        plan_affIndices(i).num_ees=plan_affIndices(i).num_ees+1;
                        plan_affIndices(i).ee_name=[plan_affIndices(i).ee_name;java.lang.String('r_hand')];
                        plan_affIndices(i).dof_name=[plan_affIndices(i).dof_name;affIndices(ind(k)).dof_name(1)];
                        plan_affIndices(i).dof_value=[plan_affIndices(i).dof_value;affIndices(ind(k)).dof_value(1)];
                    end
                    
                    if(strcmp('l_foot',ee_names{ind(k)}))
                        lfootT = ee_loci(:,ind(k));
                        l_foot_pose = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                        lfoot_const.min = l_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                        lfoot_const.max = l_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                        plan_affIndices(i).aff_type=affIndices(ind(k)).aff_type;
                        plan_affIndices(i).aff_uid=affIndices(ind(k)).aff_uid;                                                
                        plan_affIndices(i).num_ees=plan_affIndices(i).num_ees+1;
                        plan_affIndices(i).ee_name=[plan_affIndices(i).ee_name;java.lang.String('l_foot')];
                        plan_affIndices(i).dof_name=[plan_affIndices(i).dof_name;affIndices(ind(k)).dof_name(1)];
                        plan_affIndices(i).dof_value=[plan_affIndices(i).dof_value;affIndices(ind(k)).dof_value(1)];
                    end
                    
                    if(strcmp('r_foot',ee_names{ind(k)}))
                        rfootT = ee_loci(:,ind(k));
                        r_foot_pose = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                        rfoot_const.min = r_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                        rfoot_const.max = r_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                        plan_affIndices(i).aff_type=affIndices(ind(k)).aff_type;
                        plan_affIndices(i).aff_uid=affIndices(ind(k)).aff_uid;                        
                        plan_affIndices(i).num_ees=plan_affIndices(i).num_ees+1;
                        plan_affIndices(i).ee_name=[plan_affIndices(i).ee_name;java.lang.String('r_foot')];
                        plan_affIndices(i).dof_name=[plan_affIndices(i).dof_name;affIndices(ind(k)).dof_name(1)];
                        plan_affIndices(i).dof_value=[plan_affIndices(i).dof_value;affIndices(ind(k)).dof_value(1)];
                    end
                    
                end
                
                ikoptions.Q = 0*diag(cost(1:getNumDOF(obj.r)));
                ikoptions.q_nom = q_guess;
                %           0,comgoal,...
                [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                    pelvis_body,[0;0;0],pelvis_pose0,{},{},{},...
                    utorso_body,[0;0;0],utorso_pose0_relaxed,{},{},{},...
                    head_body,[0;0;0],head_pose0_relaxed,{},{},{},...
                    rfoot_body,[0;0;0],rfoot_const,{},{},{}, ...
                    lfoot_body,[0;0;0],lfoot_const,{},{},{}, ...
                    r_hand_body,[0;0;0],rhand_const,{},{},{}, ...
                    l_hand_body,[0;0;0],lhand_const,{},{},{},...
                    ikoptions);
                q_guess =q(:,i);
                toc;
                if(snopt_info == 13)
                    warning(['The IK fails at ',num2str(i)]);
                end
                %q_d(:,i) = q(ind,i);
            end
            
            % publish robot map
            disp('Publishing manip map...');
            xtraj = zeros(getNumStates(obj.r),length(affIndicesTimes));
            xtraj(1:getNumDOF(obj.r),:) = q;
            utime = now() * 24 * 60 * 60;
            obj.map_pub.publish(xtraj,plan_affIndices,utime);
        end
       
        function runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,is_keyframe_constraint)
            disp('Generating plan...');
            send_status(3,0,0,'Generating manip plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0);
            r_foot_body = obj.r.findLink('r_foot');
            l_foot_body = obj.r.findLink('l_foot');
            
            %r_foot_pose0 = forwardKin(obj.r,kinsol,r_foot_body,...
            %    r_foot_body.contact_pts(:,[r_foot_body.collision_group{1}]),2);
            %r_foot_pose0 = mean(r_foot_pose0,2);
            %l_foot_pose0 = forwardKin(obj.r,kinsol,l_foot_body,...
            %    l_foot_body.contact_pts(:,[l_foot_body.collision_group{1}]),2);
            %l_foot_pose0 = mean(l_foot_pose0,2);
            r_foot_pose0 = forwardKin(obj.r,kinsol,r_foot_body,[0;0;0],2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,l_foot_body,[0;0;0],2);
            
            % compute fixed COM goal
            gc = contactPositions(obj.r,q0);
            k = convhull(gc(1:2,:)');
            com0 = getCOM(obj.r,q0);
            %   comgoal = [mean(gc(1:2,k),2);com0(3)];
            %   comgoal = com0; % DOnt move com for now as this is pinned manipulation
            
            % get hand positions
            r_hand_body = findLink(obj.r,'r_hand');
            l_hand_body = findLink(obj.r,'l_hand');
            
            % compute EE trajectories
            r_hand_pose0 = forwardKin(obj.r,kinsol,r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,l_hand_body,[0;0;0],2);
            
            % Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            T_hand_palm_l = HT([0;0.1;0],1.57079,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],-1.57079,0,-1.57079);
            T_palm_hand_r = inv_HT(T_hand_palm_r);
            
            
            % Get head position
            head_body = findLink (obj.r, 'head');
            
            head_pose0 = forwardKin(obj.r,kinsol,head_body,[0;0;0],2);
            
            %======================================================================================================
            
            if(~is_keyframe_constraint)
                

                 obj.restrict_feet=true; 

                if(isempty(rh_ee_goal))
                    rh_ee_goal = forwardKin(obj.r,kinsol,r_hand_body,[0;0;0],1);
                    rhandT  = rh_ee_goal(1:6);
                    rhandT = [nan;nan;nan;nan;nan;nan];
                else
                    rhandT = zeros(6,1);
                    % Desired position of palm in world frame
                    T_world_palm_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
                    T_world_hand_r = T_world_palm_r*T_palm_hand_r;
                    rhandT(1:3) = T_world_hand_r(1:3,4);
                    rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                end
                
                if(isempty(lh_ee_goal))
                    lh_ee_goal = forwardKin(obj.r,kinsol,l_hand_body,[0;0;0],1);
                    lhandT  = lh_ee_goal(1:6);
                    lhandT = [nan;nan;nan;nan;nan;nan];
                else
                    lhandT = zeros(6,1);
                    % Desired position of palm in world frame
                    T_world_palm_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
                    T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                    lhandT(1:3) = T_world_hand_l(1:3,4);
                    lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                end
                
                if(isempty(rf_ee_goal))
                    rf_ee_goal = forwardKin(obj.r,kinsol,r_foot_body,[0;0;0],1);
                    rfootT  = rf_ee_goal(1:6);
                    % rfootT = [nan;nan;nan;nan;nan;nan];
                else
                    obj.restrict_feet=false;
                    rfootT = zeros(6,1);
                    % Desired position of right foot in world frame
                    T_world_foot_r = HT(rf_ee_goal(1:3),rf_ee_goal(4),rf_ee_goal(5),rf_ee_goal(6));
                    rfootT(1:3) = T_world_foot_r(1:3,4);
                    rfootT(4:6) =rotmat2rpy(T_world_foot_r(1:3,1:3));
                end
                
                if(isempty(lf_ee_goal))
                    lf_ee_goal = forwardKin(obj.r,kinsol,l_foot_body,[0;0;0],1);
                    lfootT  = lf_ee_goal(1:6);
                    %lfootT = [nan;nan;nan;nan;nan;nan];
                else
                    obj.restrict_feet=false;
                    lfootT = zeros(6,1);
                    % Desired position of left foot in world frame
                    T_world_foot_l = HT(lf_ee_goal(1:3),lf_ee_goal(4),lf_ee_goal(5),lf_ee_goal(6));
                    lfootT(1:3) = T_world_foot_l(1:3,4);
                    lfootT(4:6) =rotmat2rpy(T_world_foot_l(1:3,1:3));
                end
                
                if(isempty(h_ee_goal))
                    h_ee_goal = forwardKin(obj.r,kinsol,head_body,[0;0;0],1);
                    headT  = h_ee_goal(1:6);
                else
                    headT = zeros(6,1);
                    % Desired position of head in world frame
                    T_world_head = HT(h_ee_goal(1:3),h_ee_goal(4),h_ee_goal(5),h_ee_goal(6));
                    headT(1:3) = T_world_head(1:3, 4);
                    headT(4:6) =rotmat2rpy(T_world_head(1:3,1:3));
                end
                
                r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                r_foot_poseT = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                l_foot_poseT = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                head_poseT = [headT(1:3); rpy2quat(headT(4:6))];
                obj.rhandT = rhandT;
                obj.lhandT = lhandT;
                obj.rfootT = rfootT;
                obj.lfootT = lfootT;
                obj.headT = headT;
            else
                
                rhandT =  obj.rhandT;
                lhandT =  obj.lhandT;
                rfootT =  obj.rfootT;
                lfootT =  obj.lfootT;
                headT  =  obj.headT;
                r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                r_foot_poseT = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                l_foot_poseT = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                head_poseT = [headT(1:3); rpy2quat(headT(4:6))];
                
                if(isempty(rh_ee_goal))
                    s_int_rh= nan;
                    rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                else
                    rhand_int_constraint = zeros(6,1);
                    s_int_rh = rh_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(rh_ee_goal.desired_pose(4:7));
                    T_world_palm_r = HT(rh_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                    T_world_hand_r = T_world_palm_r*T_palm_hand_r;
                    rhand_int_constraint(1:3) = T_world_hand_r(1:3,4);
                    rhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));

                    if(abs(1-s_int_rh)<1e-3)
                      disp('rh end state is modified')
                      r_hand_poseT(1:3) = rhand_int_constraint(1:3);
                      r_hand_poseT(4:7) = rpy2quat(rhand_int_constraint(4:6));
                      obj.rhandT = rhand_int_constraint;    
                      rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                      rh_ee_goal=[];
                    end                    
                end
                
                if(isempty(lh_ee_goal))
                    s_int_lh = nan;
                    lhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                else
                    lhand_int_constraint = zeros(6,1);
                    s_int_lh = lh_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(lh_ee_goal.desired_pose(4:7));
                    T_world_palm_l = HT(lh_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                    T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                    lhand_int_constraint(1:3) = T_world_hand_l(1:3,4);
                    lhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                    
                    if(abs(1-s_int_lh)<1e-3)
                      disp('lh end state is modified')
                      l_hand_poseT(1:3) = lhand_int_constraint(1:3);
                      l_hand_poseT(4:7) = rpy2quat(lhand_int_constraint(4:6));
                      obj.lhandT = lhand_int_constraint;        
                      lhand_int_constraint = [nan;nan;nan;nan;nan;nan];
                      lh_ee_goal=[];
                    end
                end
                
                if(isempty(rf_ee_goal))
                    s_int_rf= nan;
                    rfoot_int_constraint = [nan;nan;nan;nan;nan;nan];
                else
                    rfoot_int_constraint = zeros(6,1);
                    s_int_rf = rf_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(rf_ee_goal.desired_pose(4:7));
                    T_world_foot_r = HT(rf_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                    rfoot_int_constraint(1:3) = T_world_foot_r(1:3,4);
                    rfoot_int_constraint(4:6) =rotmat2rpy(T_world_foot_r(1:3,1:3));

                    if(abs(1-s_int_rf)<1e-3)
                      disp('rf end state is modified')
                      r_foot_poseT(1:3) = rfoot_int_constraint(1:3);
                      r_foot_poseT(4:7) = rpy2quat(rfoot_int_constraint(4:6));
                      obj.rfootT = rfoot_int_constraint;    
                      rfoot_int_constraint = [nan;nan;nan;nan;nan;nan];
                      rf_ee_goal=[];            
                    end              
                end
                
                if(isempty(lf_ee_goal))
                    s_int_lf= nan;
                    lfoot_int_constraint = [nan;nan;nan;nan;nan;nan];
                else
                    lfoot_int_constraint = zeros(6,1);
                    s_int_lf = lf_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of left foot in world frame
                    rpy = quat2rpy(lf_ee_goal.desired_pose(4:7));
                    T_world_foot_l = HT(lf_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                    lfoot_int_constraint(1:3) = T_world_foot_l(1:3,4);
                    lfoot_int_constraint(4:6) =rotmat2rpy(T_world_foot_l(1:3,1:3));
                    
                    if(abs(1-s_int_lf)<1e-3)
                      disp('lf end state is modified')
                      l_foot_poseT(1:3) = lfoot_int_constraint(1:3);
                      l_foot_poseT(4:7) = rpy2quat(lfoot_int_constraint(4:6));
                      obj.lfootT = lfoot_int_constraint;    
                      lfoot_int_constraint = [nan;nan;nan;nan;nan;nan];
                      lf_ee_goal=[];             
                    end 
                end
                
                if(isempty(h_ee_goal))
                    s_int_head= nan;
                    head_int_constraint = [nan;nan;nan;nan;nan;nan];
                else
                    head_int_constraint = zeros(6,1);
                    s_int_head = h_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(h_ee_goal.desired_pose(4:7));
                    T_world_head = HT(h_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
                    head_int_constraint(1:3) = T_world_head(1:3,4);
                    head_int_constraint(4:6) =rotmat2rpy(T_world_head(1:3,1:3));
                    
                    fprintf (1, 'HEAD: Going from POS: (%f,%f,%f) --> (%f,%f,%f) and RPY: (%f,%f,%f) --> (%f,%f,%f)', ...
                             h_ee_goal.desired_pos(1), ...
                             h_ee_goal.desired_pos(2),h_ee_goal.desired_pos(3), ...
                             head_int_constraint(1), ...
                             head_int_constraint(2), ...
                             head_int_constraint(3), rpy(1), rpy(2), ...
                             rpy(3), head_int_constraint(4), ...
                             head_int_constraint(5), head_int_constraint(6));
                    
                    
                    if(abs(1-s_int_lf)<1e-3)
                      disp('lf end state is modified')
                      head_poseT(1:3) = head_int_constraint(1:3);
                      head_poseT(4:7) = rpy2quat(head_int_constraint(4:6));
                      obj.headT = head_int_constraint;    
                      head_int_constraint = [nan;nan;nan;nan;nan;nan];
                      h_ee_goal=[];             
                    end 
                end
                
                r_hand_pose_int = [rhand_int_constraint(1:3); rpy2quat(rhand_int_constraint(4:6))];
                l_hand_pose_int = [lhand_int_constraint(1:3); rpy2quat(lhand_int_constraint(4:6))];
                r_foot_pose_int = [rfoot_int_constraint(1:3); rpy2quat(rfoot_int_constraint(4:6))];
                l_foot_pose_int = [lfoot_int_constraint(1:3); rpy2quat(lfoot_int_constraint(4:6))];
                l_foot_pose_int = [lfoot_int_constraint(1:3); rpy2quat(lfoot_int_constraint(4:6))];
                head_pose_int   = [head_int_constraint(1:3); rpy2quat(head_int_constraint(4:6))];
            end
            
            %======================================================================================================
            
            ind = getActuatedJoints(obj.r);
            cost = getCostVector2(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            if(is_keyframe_constraint)
                ikoptions.MajorIterationsLimit = 100;
            else
                ikoptions.MajorIterationsLimit = 100;
            end
            %       dofnum =  find(ismember(obj.r.getStateFrame.coordinates,'l_arm_elx')==1);
            %       ikoptions.q_nom(dofnum) = 30*(pi/180);
            %       dofnum =  find(ismember(obj.r.getStateFrame.coordinates,'r_arm_elx')==1);
            %       ikoptions.q_nom(dofnum) = 30*(pi/180);
            
            comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
            comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
            pelvis_body = findLink(obj.r,'pelvis'); % dont move pelvis
            pelvis_pose0 = forwardKin(obj.r,kinsol,pelvis_body,[0;0;0],2);
            
            utorso_body = findLink(obj.r,'utorso'); % dont move pelvis
            utorso_pose0 = forwardKin(obj.r,kinsol,utorso_body,[0;0;0],2);
            utorso_pose0_relaxed = utorso_pose0;
            %       utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-1*ones(4,1)];
            %       utorso_pose0 = utorso_pose0(1:3);
            
            s = [0 1]; % normalized arc length index
            ks = ActionSequence();
            
            % kc_com = ActionKinematicConstraint(obj.r,0,[0;0;0],comgoal,[s(1),s(end)],'com');
            % ks = ks.addKinematicConstraint(kc_com);
            % kc_rfoot = ActionKinematicConstraint(obj.r,r_foot_body,mean(r_foot_body.getContactPoints('heel'),2),r_foot_pose0,[s(1),s(end)],'rfoot_heel');
            % ks = ks.addKinematicConstraint(kc_rfoot);
            % kc_lfoot = ActionKinematicConstraint(obj.r,l_foot_body,mean(l_foot_body.getContactPoints('heel'),2),l_foot_pose0,[s(1),s(end)],'lfoot_heel');
            % ks = ks.addKinematicConstraint(kc_lfoot);
            % kc_rfoot = ActionKinematicConstraint(obj.r,r_foot_body,[0;0;0],r_foot_pose0,[s(1),s(end)],'rfoot0');
            % ks = ks.addKinematicConstraint(kc_rfoot);
            % kc_lfoot = ActionKinematicConstraint(obj.r,l_foot_body,[0;0;0],l_foot_pose0,[s(1),s(end)],'lfoot0');
            % ks = ks.addKinematicConstraint(kc_lfoot);
            

            
        
            % Constraints for feet
            r_foot_poseT_relaxed.min=r_foot_poseT-1e-3;
            r_foot_poseT_relaxed.max=r_foot_poseT+1e-3;
            l_foot_poseT_relaxed.min=l_foot_poseT-1e-3;
            l_foot_poseT_relaxed.max=l_foot_poseT+1e-3;
            
            
            if(obj.restrict_feet)
               kc_rfoot = ActionKinematicConstraint(obj.r,r_foot_body,[0;0;0],r_foot_pose0,[s(1),s(end)],'rfoot0');
               ks = ks.addKinematicConstraint(kc_rfoot);
               kc_lfoot = ActionKinematicConstraint(obj.r,l_foot_body,[0;0;0],l_foot_pose0,[s(1),s(end)],'lfoot0');
               ks = ks.addKinematicConstraint(kc_lfoot);
            else
                kc_rfoot0 = ActionKinematicConstraint(obj.r,r_foot_body,[0;0;0],r_foot_pose0,[s(1),s(1)],'rfoot0');
                ks = ks.addKinematicConstraint(kc_rfoot0);
                kc_lfoot0 = ActionKinematicConstraint(obj.r,l_foot_body,[0;0;0],l_foot_pose0,[s(1),s(1)],'lfoot0');
                ks = ks.addKinematicConstraint(kc_lfoot0);
                kc_rfootT = ActionKinematicConstraint(obj.r,r_foot_body,[0;0;0],r_foot_poseT_relaxed,[s(end),s(end)],'rfootT');
                ks = ks.addKinematicConstraint(kc_rfootT);
                kc_lfootT = ActionKinematicConstraint(obj.r,l_foot_body,[0;0;0],l_foot_poseT_relaxed,[s(end),s(end)],'lfootT');
                ks = ks.addKinematicConstraint(kc_lfootT);
            end
            
            
            
            
            % Constraints for hands
            kc_rhand0 = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_pose0,[s(1),s(1)],'rhand0');
            ks = ks.addKinematicConstraint(kc_rhand0);
            
            r_hand_poseT_relaxed.min=r_hand_poseT-1e-3;
            r_hand_poseT_relaxed.max=r_hand_poseT+1e-3;
            l_hand_poseT_relaxed.min=l_hand_poseT-1e-3;
            l_hand_poseT_relaxed.max=l_hand_poseT+1e-3;
            
            kc_rhandT = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_poseT_relaxed,[s(end),s(end)],'rhandT');
            ks = ks.addKinematicConstraint(kc_rhandT); 
            kc_lhand0 = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_pose0,[s(1),s(1)],'lhand0');
            ks = ks.addKinematicConstraint(kc_lhand0); 
            kc_lhandT = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_poseT_relaxed,[s(end),s(end)],'lhandT');
            ks = ks.addKinematicConstraint(kc_lhandT); 
            

            % Constraints for head
            kc_head0 = ActionKinematicConstraint(obj.r,head_body,[0;0;0],head_pose0,[s(1),s(1)],'head0');
            ks = ks.addKinematicConstraint(kc_head0);
            
            head_poseT_relaxed.min = head_poseT-1e-3;
            head_poseT_relaxed.max = head_poseT+1e-3;
            
            kc_headT = ActionKinematicConstraint(obj.r, head_body,[0;0;0],head_poseT_relaxed,[s(end),s(end)],'headT');
            ks = ks.addKinematicConstraint(kc_headT); 
            
            kc_pelvis = ActionKinematicConstraint(obj.r,pelvis_body,[0;0;0],pelvis_pose0,[s(1),s(end)],'pelvis');
            ks = ks.addKinematicConstraint(kc_pelvis);
            kc_torso = ActionKinematicConstraint(obj.r,utorso_body,[0;0;0],utorso_pose0_relaxed,[s(1),s(end)],'utorso');
            ks = ks.addKinematicConstraint(kc_torso);
            
            if(is_keyframe_constraint)
                % If break point is adjusted via gui.
                
                r_hand_pose_int_relaxed.min= r_hand_pose_int-1e-3;
                r_hand_pose_int_relaxed.max= r_hand_pose_int+1e-3;
                l_hand_pose_int_relaxed.min= l_hand_pose_int-1e-3;
                l_hand_pose_int_relaxed.max= l_hand_pose_int+1e-3;
                r_foot_pose_int_relaxed.min= r_foot_pose_int-1e-3;
                r_foot_pose_int_relaxed.max= r_foot_pose_int+1e-3;
                l_foot_pose_int_relaxed.min= l_foot_pose_int-1e-3;
                l_foot_pose_int_relaxed.max= l_foot_pose_int+1e-3;
                head_pose_int_relaxed.min= head_pose_int-1e-3;
                head_pose_int_relaxed.max= head_pose_int+1e-3;
                
                %if((~isempty(rh_ee_goal))&&(abs(1-s_int_rh)>1e-3))
                if(~isempty(rh_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_rh)); % snap to closest break point (avoiding very close double constraints)
                    s_int_rh=obj.s_breaks(ind);
                    kc_rhand_intermediate = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_pose_int,[s_int_rh,s_int_rh],'rhand_int');
                    ks = ks.addKinematicConstraint(kc_rhand_intermediate);
                end
                %if((~isempty(lh_ee_goal))&&(abs(1-s_int_lh)>1e-3))
                if(~isempty(lh_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_lh));
                    s_int_lh=obj.s_breaks(ind);
                    kc_lhand_intermediate = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_pose_int,[s_int_lh,s_int_lh],'lhand_int');
                    ks = ks.addKinematicConstraint(kc_lhand_intermediate);
                end
                %if((~isempty(rf_ee_goal))&&(abs(1-s_int_rf)>1e-3))
                if(~isempty(rf_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_rf)); % snap to closest break point (avoiding very close double constraints)
                    s_int_rf=obj.s_breaks(ind);
                    kc_rfoot_intermediate = ActionKinematicConstraint(obj.r,r_foot_body,[0;0;0],r_foot_pose_int,[s_int_rf,s_int_rf],'rfoot_int');
                    ks = ks.addKinematicConstraint(kc_rfoot_intermediate);
                end
                %if((~isempty(lf_ee_goal))&&(abs(1-s_int_lf)>1e-3))
                if(~isempty(lf_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_lf));
                    s_int_lf=obj.s_breaks(ind);
                    kc_lfoot_intermediate = ActionKinematicConstraint(obj.r,l_foot_body,[0;0;0],l_foot_pose_int,[s_int_lf,s_int_lf],'lfoot_int');
                    ks = ks.addKinematicConstraint(kc_lfoot_intermediate);
                end
                if(~isempty(h_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_head));
                    s_int_head=obj.s_breaks(ind);
                    kc_head_intermediate = ActionKinematicConstraint(obj.r,head_body,[0;0;0],head_pose_int,[s_int_head,s_int_head],'head_int');
                    ks = ks.addKinematicConstraint(kc_head_intermediate);
                end
                
            end
            
            % Solve IK at final pose and pass as input to sequence search
            
            if(~is_keyframe_constraint)
                rhand_const.min = r_hand_poseT-1e-3;
                rhand_const.max = r_hand_poseT+1e-3;
                lhand_const.min = l_hand_poseT-1e-3;
                lhand_const.max = l_hand_poseT+1e-3;
                rfoot_const.min = r_foot_poseT-1e-3;
                rfoot_const.max = r_foot_poseT+1e-3;
                lfoot_const.min = l_foot_poseT-1e-3;
                lfoot_const.max = l_foot_poseT+1e-3;
                head_const.min = head_poseT-1e-3;
                head_const.max = head_poseT+1e-3;

                %============================
                %       0,comgoal,...
                %   [q_final_guess,snopt_info] = inverseKin(obj.r,q0,...
                %       pelvis_body,[0;0;0],pelvis_pose0,...
                %       utorso_body,[0;0;0],utorso_pose0_relaxed,...
                %       r_foot_body,'heel',r_foot_pose0, ...
                %       l_foot_body,'heel',l_foot_pose0, ...
                %       r_hand_body,[0;0;0],rhand_const, ...
                %       l_hand_body,[0;0;0],lhand_const,...
                %       ikoptions);
                if(obj.restrict_feet)
                 [q_final_guess,snopt_info] = inverseKin(obj.r,q0,...
                     pelvis_body,[0;0;0],pelvis_pose0,{},{},{},...
                     utorso_body,[0;0;0],utorso_pose0_relaxed,{},{},{},...
                     r_foot_body,[0;0;0],r_foot_pose0,{},{},{}, ...
                     l_foot_body,[0;0;0],l_foot_pose0,{},{},{}, ...
                     r_hand_body,[0;0;0],rhand_const,{},{},{}, ...
                     l_hand_body,[0;0;0],lhand_const,{},{},{},...
                     head_body,[0;0;0],head_const,{},{},{},...
                     ikoptions);
                else
                [q_final_guess,snopt_info] = inverseKin(obj.r,q0,...
                    pelvis_body,[0;0;0],pelvis_pose0,{},{},{},...
                    utorso_body,[0;0;0],utorso_pose0_relaxed,{},{},{},...
                    r_foot_body,[0;0;0],rfoot_const,{},{},{}, ...
                    l_foot_body,[0;0;0],lfoot_const,{},{},{}, ...
                    r_hand_body,[0;0;0],rhand_const,{},{},{}, ...
                    l_hand_body,[0;0;0],lhand_const,{},{},{},...
                    head_body,[0;0;0],head_const,{},{},{},...
                    ikoptions);
                end
                %============================
                
                if(snopt_info == 13)
                    warning('The IK fails at the end');
                    send_status(3,0,0,'snopt_info == 13. Manip plan initial IK is not very good.');
                end
                
                s_breaks=[s(1) s(end)];
                q_breaks=[q0 q_final_guess];
                qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_final_guess]));
                
                
            end
            
            % PERFORM IKSEQUENCE OPT
            ikseq_options.Q = diag(cost(1:getNumDOF(obj.r)));
            ikseq_options.Qa = eye(getNumDOF(obj.r));
            ikseq_options.Qv = eye(getNumDOF(obj.r));
            ikseq_options.nSample = obj.num_breaks-1;
            ikseq_options.qdotf.lb = zeros(obj.r.getNumDOF(),1);
            ikseq_options.qdotf.ub = zeros(obj.r.getNumDOF(),1);
            ikseq_options.quasiStaticFlag=false;
            if(is_keyframe_constraint)
                ikseq_options.MajorIterationsLimit = 100;
                ikseq_options.qtraj0 = obj.qtraj_guess_fine; % use previous optimization output as seed
            else
                ikseq_options.MajorIterationsLimit = 100;
                ikseq_options.qtraj0 = qtraj_guess;
            end
            %============================
            [s_breaks,q_breaks,qdos_breaks,qddos_breaks,snopt_info] = inverseKinSequence(obj.r,q0,0*q0,ks,ikseq_options);
            if(snopt_info == 13)
                warning('The IK sequence fails');
                send_status(3,0,0,'snopt_info == 13. The IK sequence fails.');
            end
            %============================
            xtraj = PPTrajectory(pchipDeriv(s_breaks,[q_breaks;qdos_breaks],[qdos_breaks;qddos_breaks]));
            xtraj = xtraj.setOutputFrame(obj.r.getStateFrame()); %#ok<*NASGU>
            %%%v = obj.r.constructVisualizer();%v.playback(xtraj,struct('slider',true)); %keyboard;
            %obj.plan_pub.publish(s_breaks,[q_breaks;qdos_breaks]);
            
            obj.s_breaks = s_breaks;
            obj.q_breaks = q_breaks;
            obj.qdos_breaks = qdos_breaks;
            
            qtraj_guess = PPTrajectory(spline(s_breaks,q_breaks));
            obj.qtraj_guess = qtraj_guess; % cache
            
            % calculate end effectors breaks via FK.
            for brk =1:length(s_breaks),
                kinsol_tmp = doKinematics(obj.r,q_breaks(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp, ...
                                                l_foot_body,[0;0;0],2);
                head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,head_body,[0;0;0],2);
            end
            
            q = q_breaks(:,1);
            q_d = q(ind);
            
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head);
            
            res = 0.1; % 10cm res
            s= linspace(0,1,round(s_total/res));
            s = unique([s(:);s_breaks(:)]);
            
            for i=2:length(s)
                si = s(i);
                %tic;
                ikoptions.Q = 0*diag(cost(1:getNumDOF(obj.r)));
                ikoptions.q_nom = q(:,i-1);
                r_hand_pose_at_t=pose_spline(s_breaks,rhand_breaks,si);  %#ok<*PROP> % evaluate in quaternions
                l_hand_pose_at_t=pose_spline(s_breaks,lhand_breaks,si); % evaluate in quaternions
                r_foot_pose_at_t=pose_spline(s_breaks,rfoot_breaks,si);  %#ok<*PROP> % evaluate in quaternions
                l_foot_pose_at_t=pose_spline(s_breaks,lfoot_breaks,si); ...
                % evaluate in quaternions
                head_pose_at_t=pose_spline(s_breaks,head_breaks,si); % evaluate in quaternions
                % r_hand_pose_at_t=pose_foh(s_breaks,rhand_breaks,si); % evaluate in quaternions
                % l_hand_pose_at_t=pose_foh(s_breaks,lhand_breaks,si); % evaluate in quaternions
                
                
                if(si~=s(end))
                    rhand_const.min = r_hand_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
                    rhand_const.max = r_hand_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
                    lhand_const.min = l_hand_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
                    lhand_const.max = l_hand_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
                    rfoot_const.min = r_foot_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
                    rfoot_const.max = r_foot_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
                    lfoot_const.min = l_foot_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
                    lfoot_const.max = l_foot_pose_at_t+1e-2* ...
                        [ones(3,1);2*ones(4,1)];
                    head_const.min = head_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
                    head_const.max = head_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
                else
                    rhand_const.min = r_hand_pose_at_t-1e-4*[ones(3,1);ones(4,1)];
                    rhand_const.max = r_hand_pose_at_t+1e-4*[ones(3,1);ones(4,1)];
                    lhand_const.min = l_hand_pose_at_t-1e-4*[ones(3,1);ones(4,1)];
                    lhand_const.max = l_hand_pose_at_t+1e-4*[ones(3,1);ones(4,1)];
                    rfoot_const.min = r_foot_pose_at_t-1e-4*[ones(3,1);2*ones(4,1)];
                    rfoot_const.max = r_foot_pose_at_t+1e-4*[ones(3,1);2*ones(4,1)];
                    lfoot_const.min = l_foot_pose_at_t-1e-4*[ones(3,1);2*ones(4,1)];
                    lfoot_const.max = l_foot_pose_at_t+1e-4* ...
                        [ones(3,1);2*ones(4,1)];
                    head_const.min = head_pose_at_t-1e-4*[ones(3,1);2*ones(4,1)];
                    head_const.max = head_pose_at_t+1e-4*[ones(3,1);2*ones(4,1)];
                end
                %l_pose_constraint=[nan;nan;nan;quat2rpy(l_hand_pose_at_t(4:7))];
                %l_pose_constraint=[nan;nan;nan;lhandT(4);lhandT(5);lhandT(6)];
                %l_pose_constraint=[lhandT(1);lhandT(2);lhandT(3);lhandT(4);lhandT(5);lhandT(6)];
                
                
                %============================
                q_guess =qtraj_guess.eval(si);
                %   0,comgoal,...
                % [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                %     pelvis_body,[0;0;0],pelvis_pose0,...
                %     utorso_body,[0;0;0],utorso_pose0_relaxed,...
                %     r_foot_body,'heel',r_foot_pose0, ...
                %     l_foot_body,'heel',l_foot_pose0, ...
                %     r_hand_body,[0;0;0],rhand_const, ...
                %     l_hand_body,[0;0;0],lhand_const,...
                %     ikoptions);
                 if(obj.restrict_feet)
                    [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                       pelvis_body,[0;0;0],pelvis_pose0,{},{},{},...
                       utorso_body,[0;0;0],utorso_pose0_relaxed,{},{},{},...
                       r_foot_body,[0;0;0],r_foot_pose0,{},{},{}, ...
                       l_foot_body,[0;0;0],l_foot_pose0,{},{},{}, ...
                       r_hand_body,[0;0;0],rhand_const,{},{},{}, ...
                       l_hand_body,[0;0;0],lhand_const,{},{},{},...
                       head_body,[0;0;0],head_const,{},{},{},...
                       ikoptions);
                 else
                    [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                        pelvis_body,[0;0;0],pelvis_pose0,{},{},{},...
                        utorso_body,[0;0;0],utorso_pose0_relaxed,{},{},{},...
                        r_foot_body,[0;0;0],rfoot_const,{},{},{}, ...
                        l_foot_body,[0;0;0],lfoot_const,{},{},{}, ...
                        r_hand_body,[0;0;0],rhand_const,{},{},{}, ...
                        l_hand_body,[0;0;0],lhand_const,{},{},{},...
                        head_body,[0;0;0],head_const,{},{},{},...
                        ikoptions);
                 end
                %============================
                
                %toc;
                if(snopt_info == 13)
                    warning(['The IK fails at ',num2str(s(i))]);
                    send_status(3,0,0,['snopt_info == 13: The IK fails at ',num2str(s(i))]);
                end
                q_d(:,i) = q(ind,i);
            end
            
            qtraj_guess_fine = PPTrajectory(spline(s, q));
            obj.qtraj_guess_fine = qtraj_guess_fine; % cache
            
            %   qd_frame = AtlasPositionRef(obj.r);
            %   des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);
            %
            % publish robot plan
            disp('Publishing plan...');
            xtraj = zeros(getNumStates(obj.r)+1,length(s));
            xtraj(1,:) = 0*s;
            for l =1:length(s_breaks),
                ind = find(s == s_breaks(l));
                xtraj(1,ind) = 1.0;
            end
            xtraj(2:getNumDOF(obj.r)+1,:) = q;
            
            
            v_desired = 0.3; % 10cm/sec seconds, hard coded for now
            ts = s.*(s_total/v_desired); % plan timesteps
            obj.time_2_index_scale = (v_desired/s_total);
            obj.plan_pub.publish(ts,xtraj);
        end
         
        function cost = getCostVector2(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 1;
            cost.base_y = 1;
            cost.base_z = 1;
            cost.base_roll = 1;
            cost.base_pitch = 1;
            cost.base_yaw = 1;
            cost.back_lbz = 1;
            cost.back_mby = 1;
            cost.back_ubx = 1;
            cost.neck_ay =  1;
            cost.l_arm_usy = 1;
            cost.l_arm_shx = 1;
            cost.l_arm_ely = 1;
            cost.l_arm_elx = 1;
            cost.l_arm_uwy = 1;
            cost.l_arm_mwx = 1;
            cost.l_leg_uhz = 1;
            cost.l_leg_mhx = 1;
            cost.l_leg_lhy = 1;
            cost.l_leg_kny = 1;
            cost.l_leg_uay = 1;
            cost.l_leg_lax = 1;
            cost.r_arm_usy = cost.l_arm_usy;
            cost.r_arm_shx = cost.l_arm_shx;
            cost.r_arm_ely = cost.l_arm_ely;
            cost.r_arm_elx = cost.l_arm_elx;
            cost.r_arm_uwy = cost.l_arm_uwy;
            cost.r_arm_mwx = cost.l_arm_mwx;
            cost.r_leg_uhz = cost.l_leg_uhz;
            cost.r_leg_mhx = cost.l_leg_mhx;
            cost.r_leg_lhy = cost.l_leg_lhy;
            cost.r_leg_kny = cost.l_leg_kny;
            cost.r_leg_uay = cost.l_leg_uay;
            cost.r_leg_lax = cost.l_leg_lax;
            cost = double(cost);
            
        end
        function cost = getCostVector(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 10000;
            cost.base_y = 10000;
            cost.base_z = 10000;
            cost.base_roll = 10000;
            cost.base_pitch = 10000;
            cost.base_yaw = 10000;
            cost.back_lbz = 10000;
            cost.back_mby = 10000;
            cost.back_ubx = 10000;
            cost.neck_ay =  10;
            cost.l_arm_usy = 1;
            cost.l_arm_shx = 1;
            cost.l_arm_ely = 1;
            cost.l_arm_elx = 1;
            cost.l_arm_uwy = 1;
            cost.l_arm_mwx = 1;
            cost.l_leg_uhz = 10000;
            cost.l_leg_mhx = 10000;
            cost.l_leg_lhy = 10000;
            cost.l_leg_kny = 10000;
            cost.l_leg_uay = 10000;
            cost.l_leg_lax = 10000;
            cost.r_arm_usy = cost.l_arm_usy;
            cost.r_arm_shx = cost.l_arm_shx;
            cost.r_arm_ely = cost.l_arm_ely;
            cost.r_arm_elx = cost.l_arm_elx;
            cost.r_arm_uwy = cost.l_arm_uwy;
            cost.r_arm_mwx = cost.l_arm_mwx;
            cost.r_leg_uhz = cost.l_leg_uhz;
            cost.r_leg_mhx = cost.l_leg_mhx;
            cost.r_leg_lhy = cost.l_leg_lhy;
            cost.r_leg_kny = cost.l_leg_kny;
            cost.r_leg_uay = cost.l_leg_uay;
            cost.r_leg_lax = cost.l_leg_lax;
            cost = double(cost);
            
        end
        
    end
    
    methods (Static=true)
        
    end
    
end
