classdef WholeBodyPlanner < KeyframePlanner
    % USAGE
    % WholeBodyPlanner wholeBodyPlanner(r);
    % wholeBodyPlanner.generateAndPublishPlan(vargin);
    % cache = wholeBodyPlanner.getPlanCache();
    %
    properties
        plan_pub        
        restrict_feet
    end
    
    methods
        function obj = WholeBodyPlanner(r,hardware_mode)
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            if(obj.isBDIManipMode())
                warning('WholeBodyPlanner:: only relevant in BDI_USER_MODE and SIM_MODE, not in BDI_MANIP_MODE.')
            end
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.restrict_feet=true;
            
            obj.plan_cache.num_breaks = 1;
            obj.plan_cache.v_desired = 0.1; % 10cm/sec seconds
        end
        
        function generateAndPublishWholeBodyPlan(obj,varargin)
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
                    runOptimizationForWholeBodyPlanGivenEELoci(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint,goal_type_flags);
                otherwise
                    error('Incorrect usage of generateAndPublishWholeBodyPlan in Manip Planner. Undefined number of vargin.')
            end
        end
        
        function runOptimizationForWholeBodyPlanGivenEELoci(obj,x0,ee_names,ee_loci,Indices,postureconstraint,goal_type_flags)
            
            disp('Generating whole body plan...');
            send_status(3,0,0,'Generating whole body plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            
            
            % Hand Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            T_hand_palm_l = HT([0;0.1;0],0,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],0,0,-1.57079);
            T_palm_hand_r = inv_HT(T_hand_palm_r);
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
            kinsol = doKinematics(obj.r,q0);
            
            % Solve IK for each element i n EE LOCII
            timeIndices=[];
            timeIndices = unique(Indices);
            
            N = length(timeIndices);
            plan_Indices=[];
            q_guess =q0;
            %plan_
            lhand_const_plan = [];
            rhand_const_plan = [];
            rhand_const_plan = [];
            rhand_const_plan = [];
            
            rh_in_contact=false;
            lh_in_contact=false;
            
                            
            obj.plan_cache.ks = ActionSequence();
            for i=1:length(timeIndices),
                
                tic;
                % get current hand and foot positions
                kinsol = doKinematics(obj.r,q_guess);
                r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],2);
                l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],2);
                head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
                head_pose0(1:3)=nan(3,1); % only set head orientation not position
                head_pose0_relaxed.min=head_pose0-[1e-2*ones(3,1);1e-2*ones(4,1)];
                head_pose0_relaxed.max=head_pose0+[1e-2*ones(3,1);1e-2*ones(4,1)];
                
                % compute fixed COM goal
                gc = contactPositions(obj.r,q_guess);
                k = convhull(gc(1:2,:)');
                com0 = getCOM(obj.r,q_guess);
                %   comgoal = [mean(gc(1:2,k(1:end-1)),2);com0(3)];
                %   comgoal = com0; % DOnt move com for now as this is pinned manipulation
                comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
                comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
                
                % compute EE trajectories
                r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
                l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
                pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
                utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
                utorso_pose0_relaxed = utorso_pose0;
                utorso_pose0_relaxed.min=utorso_pose0-[0*ones(3,1);1e-2*ones(4,1)];
                utorso_pose0_relaxed.max=utorso_pose0+[0*ones(3,1);1e-2*ones(4,1)];
                % utorso_pose0 = utorso_pose0(1:3);
                
                
                %l_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                if(goal_type_flags.lh ~=2)
                    lhand_const.min = l_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                    lhand_const.max = l_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                else
                    lhand_const.type = 'gaze';
                    lhand_const.gaze_target = ee_loci(1:3,i);
                    lhand_const.gaze_axis = [1;0;0];
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
                else
                    head_const = [];
                end
                %l_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
                %                 lfoot_const.min = l_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                %                 lfoot_const.max = l_foot_pose0+1e-2*[ones(3,1);ones(4,1)];
                %r_foot_pose0= [n    an;nan;nan;nan;nan;nanT;nan];
                %                 rfoot_const.min = r_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
                %                 rfoot_const.max = r_foot_pose0+1e-2*[ones(3,1);ones(4,1)];
                r_foot_pose = r_foot_pose0;
                T_world_r_foot = [quat2rotmat(r_foot_pose(4:7)) r_foot_pose(1:3);0 0 0 1];
                r_foot_pts_pose = T_world_r_foot*[r_foot_pts;ones(1,num_r_foot_pts)];
                r_foot_pts_pose = [r_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_r_foot_pts),r_foot_pose(4:7))];
                rfoot_const.min = r_foot_pts_pose-1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                rfoot_const.max = r_foot_pts_pose+1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
                rfoot_const_static_contact = rfoot_const;
                rfoot_const_static_contact.contact_state = ...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
                
                l_foot_pose = l_foot_pose0;
                T_world_l_foot = [quat2rotmat(l_foot_pose(4:7)) l_foot_pose(1:3);0 0 0 1];
                l_foot_pts_pose = T_world_l_foot*[l_foot_pts;ones(1,num_l_foot_pts)];
                l_foot_pts_pose = [l_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_l_foot_pts),l_foot_pose(4:7))];
                lfoot_const.min = l_foot_pts_pose-1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                lfoot_const.max = l_foot_pts_pose+1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
                lfoot_const_static_contact = lfoot_const;
                lfoot_const_static_contact.contact_state = ...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
                
                ind = [];
                ind=find(Indices==timeIndices(i));

                
                % Find all active     constraints at current index
                for k=1:length(ind),
                    if(strcmp('left_palm',ee_names{ind(k)}))
                        l_ee_goal = ee_loci(:,ind(k));
                        lhandT = zeros(6,1);
                        T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                        T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                        lhandT(1:3) = T_world_hand_l(1:3,4);
                        lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                        l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                        T_world_grasp_l = T_world_palm_l * T_palm_grasp;
                        lgraspT = zeros(6,1);
                        lgraspT(1:3) = T_world_grasp_l(1:3,4);
                        lgraspT(4:6) =rotmat2rpy(T_world_grasp_l(1:3,1:3));
                        l_grasp_pose = [lgraspT(1:3); rpy2quat(lgraspT(4:6))];
                        lhand_const.min = l_hand_pose-0*1e-4*[ones(3,1);ones(4,1)];
                        lhand_const.max = l_hand_pose+0*1e-4*[ones(3,1);ones(4,1)];                         
                        lh_in_contact=true;
                    end
                    if(~lh_in_contact)
                        l_hand_pose = [nan;nan;nan;nan;nan;nan;nan];
                        lhand_const.min=l_hand_pose;
                        lhand_const.max=l_hand_pose;
                    end
                    
                    if(strcmp('right_palm',ee_names{ind(k)}))
                        r_ee_goal = ee_loci(:,ind(k));
                        rhandT = zeros(6,1);
                        T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                        T_world_hand_r = T_world_palm_r*T_palm_hand_r;
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
                        
                        rh_in_contact=true;
                    end
                    if(~rh_in_contact)
                        r_hand_pose = [nan;nan;nan;nan;nan;nan;nan];
                        rhand_const.min=r_hand_pose;
                        rhand_const.max=r_hand_pose;
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

                    end
                   
                end

               if(i==length(timeIndices))
                obj.cacheLHandPose([0 0],'lhand0',l_hand_pose0);                      
                obj.cacheLHandPose([i/N i/N],'lhandT',l_hand_pose);
                obj.cacheRHandPose([0 0],'rhand0',r_hand_pose0);       
                obj.cacheRHandPose([i/N i/N],'rhandT',r_hand_pose);
                obj.cacheRFootPoseAsContactConstraint([0 0],'rfoot0',r_foot_pose0);   
                obj.cacheRFootPoseAsContactConstraint([i/N i/N],['rfootT'],r_foot_pose);  
                obj.cacheLFootPoseAsContactConstraint([0 0],'lfoot0',l_foot_pose0);    
                obj.cacheLFootPoseAsContactConstraint([i/N i/N],['lfootT'],l_foot_pose);    
               else
                obj.cacheLHandPose([i/N i/N],['lhand' num2str(i-1)],lhand_const);  
                obj.cacheRHandPose([i/N i/N],['rhand' num2str(i-1)],rhand_const);  
                obj.cacheRFootPoseAsContactConstraint([i/N i/N],['rfoot' num2str(i-1)],r_foot_pose); 
                obj.cacheLFootPoseAsContactConstraint([i/N i/N],['lfoot' num2str(i-1)],l_foot_pose); 
               end
           
                ikoptions.jointLimitMin = obj.joint_min(1:obj.r.getNumDOF());        
                ikoptions.jointLimitMax = obj.joint_max(1:obj.r.getNumDOF());              
                ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
                ikoptions.q_nom = q_guess;                
                ikoptions.quasiStaticFlag = true;
                [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                    obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                    obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                    obj.r_hand_body,[0;0;0],rhand_const, ...
                    obj.l_hand_body,[0;0;0],lhand_const,...
                    ikoptions);
                
                q_guess =q(:,i);
                toc;
                if(snopt_info > 10)
                    msg=['The IK fails at ',num2str(i)];
                    warning(msg);
                    send_status(3,0,0,msg);
                end
                %q_d(:,i) = q(ind,i);
            end
            
            % publish robot map
            disp('Publishing manip plan...');
            
            
            utime = now() * 24 * 60 * 60;
            xtraj = zeros(getNumStates(obj.r)+2,length(timeIndices));
            xtraj(1,:) = 0*timeIndices;
            obj.plan_cache.num_breaks = 4;%length(timeIndices);
            keyframe_inds = unique(round(linspace(1,length(timeIndices),obj.plan_cache.num_breaks))); % no more than ${obj.plan_cache.num_breaks} keyframes
            xtraj(1,keyframe_inds) = 1.0;
            xtraj(2,:) = 0*timeIndices;
            xtraj(3:getNumDOF(obj.r)+2,:) = q;
            
            s = (timeIndices-min(timeIndices))/(max(timeIndices)-min(timeIndices));
            
            %timeIndices
            %fprintf('Max : %f - Min : %f', max(timeIndlices), min(timeIndices));
            
            
            
            s_sorted = sort(s);
            s_breaks = s_sorted(keyframe_inds);
            
            %Update Plan Cache
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            obj.plan_cache.s_breaks = s_breaks;
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
                %pelvis_pose= forwardKin(obj.r,kinsol_tmp,obj.pelvis_body,[0;0;0],2);
                %obj.cachePelvisPose([brk/length(s_breaks) brk/length(s_breaks)],'pelvis',pelvis_pose);
            end
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head);
            s_total = max(s_total,0.01);
            
            ts = s.*(s_total/obj.plan_cache.v_desired); % plan timesteps
            obj.plan_cache.time_2_index_scale = (obj.plan_cache.v_desired/s_total);
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
                obj.plan_pub.publish(xtraj,ts,G,utime);
            else
                obj.plan_pub.publish(xtraj,ts,utime);
            end
        end% end function
        
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
            cost.l_leg_hpz = 1;
            cost.l_leg_hpx = 1;
            cost.l_leg_hpy = 1;
            cost.l_leg_kny = 1;
            cost.l_leg_aky = 1;
            cost.l_leg_akx = 1;
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
            
        end% end function
    end% end methods
end% end classdef
