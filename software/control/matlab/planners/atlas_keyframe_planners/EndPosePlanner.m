classdef EndPosePlanner < KeyframePlanner
    % USAGE
    % EndPosePlanner endPosePlanner(r);
    % endPosePlanner.generateAndPublishCandidateRobotEndPose(vargin);
    % cache = endPosePlanner.getPlanCache();
    
    properties
        pose_pub
    end
    
    methods
        function obj = EndPosePlanner(r,hardware_mode)
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
            
            obj.plan_cache.num_breaks = 1;
            if(obj.isSimMode())
              obj.plan_cache.v_desired = 0.1; % 10cm/sec seconds
            else
              obj.plan_cache.v_desired = 0.05; % 5cm/sec seconds
            end
            % Caches a Redundant two element plan.
            % Flag indicates KeyframeAdjustmentEngine to
            % publish an single keyframe endpose instead
            % of a keyframe plan by resolving at time T.
            obj.plan_cache.isEndPose = true;
        end
        
        function generateAndPublishCandidateRobotEndPose(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags) %#ok<INUSD>
            runPoseOptimization(obj,x0,ee_names,ee_loci,timeIndices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags);
        end
        
        function runPoseOptimization(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags)
            
            disp('Generating candidate endpose...');
            send_status(3,0,0,'Generating candidate endpose...');
            
            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
            
            
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,2);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,2);
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
            r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
            
            
            % Hand Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            T_hand_palm_l = HT([0;0.1;0],0,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],0,0,-1.57079);
            T_palm_hand_r = inv_HT(T_hand_palm_r);
            
            
            cost = getCostVector2(obj);
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.MajorIterationsLimit = 1000;
            ikoptions.shrinkFactor = 0.8;
            
            
            % Solve IK
            timeIndices = unique(Indices);
            N = length(timeIndices);
            if(N>1)
                disp('Error: ERROR optimization expects constraint at a single timestamp. Cosntraints at multiple times received.');
                send_status(3,0,0,'ERROR: Pose optimization expects constraint at a single timestamp. Cosntraints at multiple times received.');
                return;
            end
            
            q_guess =q0;
            
            
            %l_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
            l_foot_pose0(1:2,:)=nan(2,num_l_foot_pts);
            r_foot_pose0(1:2,:)=nan(2,num_r_foot_pts);
            head_pose0(1:3)=nan(3,1); % only set head orientation not position
            pelvis_pose0(1:2)=nan(2,1); % The problem is to find the pelvis pose
            utorso_pose0(1:2)=nan(2,1);
            pelvis_const.min = nan(7,1);
            pelvis_const.max = nan(7,1);
            lhand_const.min = nan(7,1);
            lhand_const.max = nan(7,1);
            rhand_const.min = nan(7,1);
            rhand_const.max = nan(7,1);
            if(goal_type_flags.rh == 2)
                rhand_const.type = 'gaze';
                rhand_const.gaze_axis = [1;0;0];
                rhand_const.gaze_target = rh_ee_goal(1:3);
                rhand_const.gaze_conethreshold = pi/18;
            end
            if(goal_type_flags.lh == 2)
                lhand_const.type = 'gaze';
                lhand_const.gaze_axis = [1;0;0];
                lhand_const.gaze_target = lh_ee_goal(1:3);
                lhand_const.gaze_conethreshold = pi/18;
            end
            if(goal_type_flags.h == 2)
                head_const.type = 'gaze';
                head_const.gaze_axis = [1;0;0];
                head_const.gaze_target = h_ee_goal(1:3);
                head_const.gaze_conethreshold = pi/12;
            else
                head_const = [];
            end
            lfoot_const.min = l_foot_pose0(1:3,:)-1e-4*ones(3,num_l_foot_pts);
            lfoot_const.max = l_foot_pose0(1:3,:)+1e-4*ones(3,num_l_foot_pts);
            rfoot_const.min = r_foot_pose0(1:3,:)-1e-4*ones(3,num_r_foot_pts);
            rfoot_const.max = r_foot_pose0(1:3,:)+1e-4*ones(3,num_r_foot_pts);
            rfoot_const_static_contact = rfoot_const;
            rfoot_const_static_contact.contact_state = ...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
            head_pose0_relaxed.min=head_pose0-[1e-1*ones(3,1);1e-2*ones(4,1)];
            head_pose0_relaxed.max=head_pose0+[1e-1*ones(3,1);1e-2*ones(4,1)];
            pelvis_pose0_relaxed.min=pelvis_pose0-[0*ones(3,1);1e-1*ones(4,1)];
            pelvis_pose0_relaxed.max=pelvis_pose0+[0*ones(3,1);1e-1*ones(4,1)];
            utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-2*ones(4,1)];
            utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-2*ones(4,1)];
            
            
            ind=find(Indices==timeIndices(1));
            obj.plan_cache.ks = ActionSequence();
            rhandT=[];
            for k=1:length(ind),
                if(strcmp('pelvis',ee_names{ind(k)}))
                    pelvisT = ee_loci(:,ind(k));
                    %pelvis_pose = [nan(2,1);pelvisT(3); rpy2quat(pelvisT(4:6))];
                    pelvis_pose = [nan(3,1); rpy2quat(pelvisT(4:6))];
                    pelvis_const.min = pelvis_pose-1e-4*[zeros(3,1);ones(4,1)];
                    pelvis_const.max = pelvis_pose+1e-4*[zeros(3,1);ones(4,1)];
                    %pelvis_const.min(3) = pelvis_pose(3)+0.1;
                    %pelvis_const.max(3) = pelvis_pose(3)+0.3;
                    pelvis_const.min(4:7) = pelvis_pose(4:7)-0.1*ones(4,1);
                    pelvis_const.max(4:7) = pelvis_pose(4:7)+0.1*ones(4,1);
                    %                     pelvis_const.type = 'gaze';
                    %                     pelvis_const.gaze_orientation = pelvisT(4:6);
                    % %                     pelvis_const.gaze_dir = rpy2rotmat(pelvisT(4:6))'*[0;0;1];
                    %                     pelvis_const.gaze_axis = [0;0;1];
                    pelvis_const.gaze_threshold = pi/4;
                elseif(strcmp('left_palm',ee_names{ind(k)}))
                    l_ee_goal = ee_loci(:,ind(k));
                    lhandT = zeros(6,1);
                    T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                    T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                    lhandT(1:3) = T_world_hand_l(1:3,4);
                    lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                    l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                    lhand_const.min = l_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                    lhand_const.max = l_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                    
                    name = ['lhand' num2str(k)];
                    kc_lhand = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_const,[i/N i/N],name);
                    obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lhand);
                    
                    %q_guess(1:3) = l_hand_pose(1:3);
                elseif(strcmp('right_palm',ee_names{ind(k)}))
                    r_ee_goal = ee_loci(:,ind(k));
                    rhandT = zeros(6,1);
                    T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                    T_world_hand_r = T_world_palm_r*T_palm_hand_r;
                    rhandT(1:3) = T_world_hand_r(1:3,4);
                    rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                    r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                    rhand_const.min = r_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                    rhand_const.max = r_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                    %q_guess(1:3) = r_hand_pose(1:3);
                    
                    name = ['rhand' num2str(k)];
                    kc_rhand = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_const,[i/N i/N],name);
                    obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rhand);
                    
                elseif (strcmp('l_foot',ee_names{ind(k)}))
                    lfootT = ee_loci(:,ind(k));
                    %l_foot_pose = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                    %lfoot_const.min = l_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                    %lfoot_const.max = l_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                    l_foot_pose = [lfootT(1:3)];
                    lfoot_const.min = l_foot_pose-1e-6*[ones(3,1)];
                    lfoot_const.max = l_foot_pose+1e-6*[ones(3,1)];
                    
                elseif(strcmp('r_foot',ee_names{ind(k)}))
                    rfootT = ee_loci(:,ind(k));
                    %r_foot_pose = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                    %rfoot_const.min = r_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                    %rfoot_const.max = r_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                    r_foot_pose = [rfootT(1:3)];
                    rfoot_const.min = r_foot_pose-1e-6*[ones(3,1)];
                    rfoot_const.max = r_foot_pose+1e-6*[ones(3,1)];
                else
                    disp('currently only feet/hands and pelvis are allowed');
                end
            end
            rfoot_const_static_contact = rfoot_const;
            rfoot_const_static_contact.contact_state = ...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
            lfoot_const_static_contact = lfoot_const;
            lfoot_const_static_contact.contact_state = ...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
            
            
            % cache foot or pelvis constraints for end poses depending on Hardware Mode
            if(obj.isBDIManipMode())
                kc_pelvis = ActionKinematicConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_const,[0 1],'pelvis');
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_pelvis)
            else
                % TODO: VERIFY
                kc_lfootT = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,[1,1],'lfootT',...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ContactAffordance()},...
                    {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lfootT);
                kc_rfootT = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,[1 1],'rfootT',...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ContactAffordance()},...
                    {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rfootT);
            end
            
            
            
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            
            nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_comfortable_right_arm_manip.mat'));
            qstar = nomdata.xstar(1:obj.r.getNumDOF());
            %  			ikoptions.q_nom = qstar;
            NSamples = 10;
            yaw_angles_bnd = 25;
            arm_loci_flag = ~cellfun(@(x) isempty(strfind(char(x),'palm')),ee_names);
            if(sum(arm_loci_flag) == 2)
                nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_two_hands_reaching.mat'));
                qstar = nomdata.xstar(1:obj.r.getNumDOF());
                ikoptions.q_nom = qstar;
                rhand_const.min = rhand_const.min-0.03*ones(7,1);
                rhand_const.max = rhand_const.max+0.03*ones(7,1);
                lhand_const.min = lhand_const.min-0.03*ones(7,1);
                lhand_const.max = lhand_const.max+0.03*ones(7,1);
                ikoptions.shrinkFactor = 0.95;
                cost = diag(obj.getCostVector3());
                cost = cost(1:obj.r.getNumDOF(),1:obj.r.getNumDOF());
                ikoptions.Q = cost;
                [ikoptions.jointLimitMin,ikoptions.jointLimitMax] = obj.r.getJointLimits();
                state_frame = obj.r.getStateFrame;
                ikoptions.jointLimitMin = Point(state_frame,[ikoptions.jointLimitMin;zeros(obj.r.getNumDOF,1)]);
                ikoptions.jointLimitMax = Point(state_frame,[ikoptions.jointLimitMax;zeros(obj.r.getNumDOF,1)]);
                ikoptions.jointLimitMin.back_bky = -0.3;
                ikoptions.jointLimitMax.back_bky = 0.3;
                ikoptions.jointLimitMax.r_arm_usy = 0;
                ikoptions.jointLimitMax.l_arm_usy = 0;
                ikoptions.jointLimitMin.r_leg_kny = 0.2;
                ikoptions.jointLimitMin.l_leg_kny = 0.2;
                ikoptions.jointLimitMin = double(ikoptions.jointLimitMin);
                ikoptions.jointLimitMax = double(ikoptions.jointLimitMax);
                ikoptions.jointLimitMin = ikoptions.jointLimitMin(1:obj.r.getNumDOF);
                ikoptions.jointLimitMax = ikoptions.jointLimitMax(1:obj.r.getNumDOF);
                coords = obj.r.getStateFrame.coordinates();
                coords = coords(1:obj.r.getNumDOF());
                arm_joint_ind = ~cellfun(@isempty,strfind(coords,'arm'));
                ikoptions.jointLimitMin(arm_joint_ind) = 0.9*ikoptions.jointLimitMin(arm_joint_ind);
                ikoptions.jointLimitMax(arm_joint_ind) = 0.9*ikoptions.jointLimitMax(arm_joint_ind);
                if(~isempty(h_ee_goal))
                    head_const.gaze_conethreshold = pi/4;
                end
                NSamples = 20;
                yaw_samples_bnd = 60;
            end
            for k=1:NSamples,
                %q_guess = qstar;
                q_guess(3) = q_guess(3)+2*(rand(1,1)-0.5)*(0.2);
                q_guess(6)=q_guess(6)+2*(rand(1,1)-0.5)*(yaw_angles_bnd*pi/180);%+-10degrees from current pose
                if(~obj.isBDIManipMode())
                ikoptions.quasiStaticFlag = true;
                else
                ikoptions.quasiStaticFlag = false;    
                end
                [ikoptions.jointLimitMin,ikoptions.jointLimitMax] = obj.r.getJointLimits();
                state_frame = obj.r.getStateFrame;
                ikoptions.jointLimitMin = Point(state_frame,[ikoptions.jointLimitMin;zeros(obj.r.getNumDOF,1)]);
                ikoptions.jointLimitMax = Point(state_frame,[ikoptions.jointLimitMax;zeros(obj.r.getNumDOF,1)]);
                ikoptions.jointLimitMin.back_bky = -0.3;
                ikoptions.jointLimitMax.back_bky = 0.3;
                ikoptions.jointLimitMin.r_leg_kny = 0.2;
                ikoptions.jointLimitMin.l_leg_kny = 0.2;
                ikoptions.jointLimitMin = double(ikoptions.jointLimitMin);
                ikoptions.jointLimitMax = double(ikoptions.jointLimitMax);
                ikoptions.jointLimitMin = ikoptions.jointLimitMin(1:obj.r.getNumDOF());
                ikoptions.jointLimitMax = ikoptions.jointLimitMax(1:obj.r.getNumDOF());
                %obj.pelvis_body,[0;0;0],pelvis_const,...
                %   obj.head_body,[0;0;0],head_pose0_relaxed,...
                %   obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...
                
                if(obj.isBDIManipMode()) % replace feet with pelvis in BDI Manip Mode                   
                    
                    if(~isempty(head_const))
                        [q_sample(:,k),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_const,...
                            obj.r_hand_body,[0;0;0],rhand_const,...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            obj.head_body,[0;0;0],head_const,...
                            ikoptions);
                    else
                        [q_sample(:,k),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.pelvis_body,[0;0;0],pelvis_const,...
                            obj.r_hand_body,[0;0;0],rhand_const,...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    end                   
                else
                    if(~isempty(head_const))
                        [q_sample(:,k),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                            obj.r_hand_body,[0;0;0],rhand_const,...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            obj.head_body,[0;0;0],head_const,...
                            ikoptions);
                    else
                        [q_sample(:,k),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                            obj.r_hand_body,[0;0;0],rhand_const,...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
                    end
                end
                
                if(snopt_info > 10)
                    warning(['poseOpt IK fails']);
                    send_status(4,0,0,sprintf('snopt_info = %d...',snopt_info));
                end
                if(snopt_info < 10)
                    sample_cost(:,k) = (q_sample(:,k)-ikoptions.q_nom)'*ikoptions.Q*(q_sample(:,k)-ikoptions.q_nom);
                else
                    send_status(3,0,0,'Bad candidate startpose...');
                    sample_cost(:,k) =  Inf;
                end
                disp(['sample_cost(:,k): ' num2str(sample_cost(:,k))]);
            end
            if(all(isinf(sample_cost)))
                send_status(3,0,0,'All samples are infeasible');
            end
            [~,k_best] = min(sample_cost);
            disp(['sample_cost(:,k): ' num2str(sample_cost(:,k_best))]);
            q_out = q_sample(:,k_best); % take the least cost pose
            
            % publish robot pose
            disp('Publishing candidate endpose ...');
            %            kinsol_out = doKinematics(obj.r,q_out);
            %            [~,J_rh] = forwardKin(obj.r,kinsol_out,obj.r_hand_body,[0;0;0],1);
            %            fprintf('The condition number of Jacobian matrix of the right hand is %10.4f\n',cond(J_rh));
            send_status(3,0,0,'Publishing candidate endpose...');
            utime = now() * 24 * 60 * 60;
            xtraj = zeros(getNumStates(obj.r),1);
            xtraj(1:getNumDOF(obj.r),:) = q_out;
            obj.pose_pub.publish(xtraj,utime);
            
            %TODO: Update Plan Cache
            s = [0 1];
            q = [q_out q_out];
            obj.plan_cache.s_breaks = s;
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            obj.plan_cache.quasiStaticFlag =  ikoptions.quasiStaticFlag;
            
        end  % end function
        
        function cost = getCostVector3(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 0;
            cost.base_y = 0;
            cost.base_z = 100;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 0;
            cost.back_bkz = 10;
            cost.back_bky = 10;
            cost.back_bkx = 10;
            cost.neck_ay =  10;
            cost.l_arm_usy = 1000;
            cost.l_arm_shx = 1000;
            cost.l_arm_ely = 50;
            cost.l_arm_elx = 50;
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
        
        function cost = getCostVector2(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 1;
            cost.base_y = 1;
            cost.base_z = 1;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 1;
            cost.back_bkz = 10;
            cost.back_bky = 10000;
            cost.back_bkx = 10000;
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
