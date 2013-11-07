classdef EndPosePlanner < KeyframePlanner
    % USAGE
    % EndPosePlanner endPosePlanner(r);
    % endPosePlanner.generateAndPublishCandidateRobotEndPose(vargin);
    % cache = endPosePlanner.getPlanCache();
    
    properties
        pose_pub
        ee_torso_dist_lb
        ee_lleg_dist_lb
        stance_lb
        stance_ub
        l_hpx_ub
        l_hpy_ub
        l_hpx_lb
        l_hpy_lb
        shrinkfactor
        pelvis_upright_gaze_tol
    end
    
    methods
        function obj = EndPosePlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode)
            addpath spherical_interp/;
            obj = obj@KeyframePlanner(r,atlas,lhand_frame,rhand_frame); % initialize the base class
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            
            
            joint_names = atlas.getStateFrame.coordinates(1:getNumDOF(atlas));
            %joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
            obj.shrinkfactor = 0.4;
            obj.plan_cache.num_breaks = 1;
            
            % Caches a Redundant two element plan.
            % Flag indicates KeyframeAdjustmentEngine to
            % publish an single keyframe endpose instead
            % of a keyframe plan by resolving at time T.
            obj.plan_cache.isEndPose = true;
            obj.ee_torso_dist_lb = 0.65;
            obj.ee_lleg_dist_lb = 0.3;
            obj.stance_lb = 0.2;
            obj.stance_ub = 0.35;
            obj.l_hpx_ub = inf;
            obj.l_hpy_ub = -0.2;
            obj.l_hpx_lb = 0.0;
            obj.l_hpy_lb = -1.0;
;
            
            obj.pelvis_upright_gaze_tol = pi/30;
        end
        %-----------------------------------------------------------------------------------------------------------------
        function generateAndPublishCandidateRobotEndPose(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags) %#ok<INUSD>
            N = length(unique(timeIndices));
            if(N>1)
                %performs IKtraj 
                 runPoseOptimizationViaMultitimeIKtraj(obj,x0,ee_names,ee_loci,timeIndices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags);
            else
                % performs IK
                runPoseOptimizationViaSingleTimeIK(obj,x0,ee_names,ee_loci,timeIndices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags);
            end
        end
        %-----------------------------------------------------------------------------------------------------------------
        function runPoseOptimizationViaMultitimeIKtraj(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags)
            disp('Generating candidate endpose via IKTraj Given EE Loci...');
            send_status(3,0,0,'Generating candidate endpose given EE Loci...');
            
            
            q0 = x0(1:getNumDOF(obj.r));
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
            
            r_foot_pts = [0;0;0];
            l_foot_pts = [0;0;0];
            r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            r_foot_contact_pts = r_foot_contact_pts(:,r_foot_contact_pts(3,:)<=mean(r_foot_contact_pts(3,:)));
            l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            l_foot_contact_pts = l_foot_contact_pts(:,l_foot_contact_pts(3,:)<=mean(l_foot_contact_pts(3,:)));
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,2);
            r_foot_contact_pos = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_contact_pts,0);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,2);
            l_foot_contact_pos = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_contact_pts,0);
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
            r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
            

            
            iktraj_lhand_constraint = {};
            iktraj_rhand_constraint = {};
            iktraj_lfoot_constraint = {};
            iktraj_rfoot_constraint = {};
            iktraj_head_constraint = {};
            iktraj_pelvis_constraint = {};   
            iktraj_dist_constraint = {};
            

            l_foot_pose0(1:2,:)=nan(2,num_l_foot_pts);
            r_foot_pose0(1:2,:)=nan(2,num_r_foot_pts);
            head_pose0(1:3)=nan(3,1); % only set head orientation not position
            pelvis_pose0(1:2)=nan(2,1); % The problem is to find the pelvis pose
            utorso_pose0(1:2)=nan(2,1);
            r_hand_pose = nan(7,1);
            l_hand_pose = nan(7,1);
            
            % gaze constraints
            tspan = [0 1];
            if(goal_type_flags.rh == 2)
                iktraj_rhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.r_hand_body,obj.rh_gaze_axis,rh_ee_goal(1:3),[0;0;0],obj.hand_gaze_tol)};
            else
                iktraj_rhand_constraint = {};
            end
            if(goal_type_flags.lh == 2)
                iktraj_lhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.l_hand_body,obj.lh_gaze_axis,lh_ee_goal(1:3),[0;0;0],obj.hand_gaze_tol)};
            else
                iktraj_lhand_constraint = {};
            end
            if(goal_type_flags.h == 2)
                iktraj_head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,h_ee_goal(1:3),[0;0;0],obj.head_gaze_tol,tspan)};
            else
                iktraj_head_constraint = {};
            end
            
            % Feet z constraints
            l_foot_pose = l_foot_pose0;
            r_foot_pose = r_foot_pose0;
            
            
            lfoot_constraint ={WorldPositionConstraint(obj.r,obj.l_foot_body,l_foot_contact_pts,...
                [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)],...
                [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)])};
            rfoot_constraint = {WorldPositionConstraint(obj.r,obj.r_foot_body,r_foot_contact_pts,...
                [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)],...
                [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)])};
            iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];     
            iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];
            
            lfoot_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.l_foot_body,[0 1])};
            rfoot_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.r_foot_body,[0 1])};
            iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];     
            iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];
            iktraj_pelvis_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.pelvis_body,[0 1]),WorldGazeDirConstraint(obj.r,obj.pelvis_body,[0;0;1],[0;0;1],obj.pelvis_upright_gaze_tol,[0 1])};

            % Solve IK
            timeIndices = unique(Indices);

            
            lh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.lh_name)),ee_names));
            rh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.rh_name)),ee_names));
            lf_indices = (~cellfun(@(x) isempty(strfind(char(x),'l_foot')),ee_names));
            rf_indices = (~cellfun(@(x) isempty(strfind(char(x),'r_foot')),ee_names));
            lhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,lh_indices));
            rhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,rh_indices));
            lfoot_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,lf_indices));
            rfoot_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,rf_indices));

            
            N = length(timeIndices);
            NBreaks = min(N,5);% No more than 5 breaks for IKTraj.
            s_breaks = linspace(0,1,NBreaks);
            s = linspace(0,1,N);
           
            
            for j=1:NBreaks,
                si=s_breaks(j);
                if(~isempty(lhand_constraints))    
                    if(j==1)
                       iktraj_lhand_constraint = {}; % flush gaze constraints       
                    end
                    l_hand_pose = pose_spline(s,lhand_constraints,si);
                    l_ee_goal = [l_hand_pose(1:3);quat2rpy(l_hand_pose(4:7))];
                    lhandT = zeros(6,1);
                    T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                    T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
                    lhandT(1:3) = T_world_hand_l(1:3,4);
                    lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                    l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                    tspan = [si si];
                    lhand_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose,1e-2,sind(2).^2,tspan);    
                    iktraj_lhand_constraint = [iktraj_lhand_constraint,lhand_constraint];   
                    
                    if(j==1)
                        dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,[0 1])};
                        iktraj_dist_constraint = [iktraj_dist_constraint,dist_constraint];       
                    end
                end    

                if(~isempty(rhand_constraints))     
                    if(j==1)
                       iktraj_rhand_constraint = {}; % flush gaze constraints       
                    end                    
                    r_hand_pose = pose_spline(s,rhand_constraints,si);
                    r_ee_goal = [r_hand_pose(1:3);quat2rpy(r_hand_pose(4:7))];
                    rhandT = zeros(6,1);
                    T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                    T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
                    rhandT(1:3) = T_world_hand_r(1:3,4);
                    rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                    r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                    tspan = [si si];
                    rhand_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose,1e-2,sind(2).^2,tspan);    
                    iktraj_rhand_constraint = [iktraj_rhand_constraint,rhand_constraint];  
                    if(j==1)
                        dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.r_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,[0 1])};
                        iktraj_dist_constraint = [iktraj_dist_constraint,dist_constraint];       
                    end
                end  

                if(~isempty(lfoot_constraints))  
                    if(j==1)
                       iktraj_lfoot_constraint = {}; % flush feet height constraint       
                    end                           
                    l_foot_pose = pose_spline(s,lfoot_constraints,si);
                    tspan = [si si];
                    lfoot_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose,1e-6,1e-6,tspan);
                    iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];      
                end    

                if(~isempty(rfoot_constraints))  
                    if(j==1)
                       iktraj_rfoot_constraint = {}; % flush feet height constraint      
                    end   
                    r_foot_pose = pose_spline(s,rfoot_constraints,si);
                    tspan = [si si];
                    rfoot_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose,1e-6,1e-6,tspan);
                    iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];      
                end  
          
                
            end
     
            
          % PERFORM IKSEQUENCE OPT
          cost = getCostVector(obj);
          iktraj_options = IKoptions(obj.r);
          iktraj_options = iktraj_options.setDebug(true);
          iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
          iktraj_options = iktraj_options.setQa(0.05*eye(getNumDOF(obj.r)));
          iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
          iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1));
          iktraj_options = iktraj_options.setFixInitialState(false);
          iktraj_options = iktraj_options.setMajorIterationsLimit(1000);
          iktraj_options = iktraj_options.setIterationsLimit(50000);
          
          qsc = QuasiStaticConstraint(obj.r);
          qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);  
          qsc = qsc.setActive(true);
          qsc = qsc.setShrinkFactor(obj.shrinkfactor); % search for a conservative pose
          iktraj_options = iktraj_options.setMajorIterationsLimit(1000);
          nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
          qstar = nomdata.xstar(1:obj.r.getNumDOF());
          iktraj_tbreaks = s_breaks;
          iktraj_qseed_traj = PPTrajectory(foh(iktraj_tbreaks,[q0 repmat(qstar,1,NBreaks-1)]));
          iktraj_qnom_traj = PPTrajectory(foh(iktraj_tbreaks,repmat(qstar,1,NBreaks)));           
          
          joint_constraint = PostureConstraint(obj.r);
          joint_constraint = joint_constraint.setJointLimits((1:obj.r.getNumDOF)',obj.joint_constraint.lb,obj.joint_constraint.ub);
          coords = obj.r.getStateFrame.coordinates;
          % urf limits are lower="-0.523599" upper="0.523599" 
          l_leg_hpx_ind = find(strcmp(coords,'l_leg_hpx'));      r_leg_hpx_ind = find(strcmp(coords,'r_leg_hpx'));
          l_leg_hpy_ind = find(strcmp(coords,'l_leg_hpy'));      r_leg_hpy_ind = find(strcmp(coords,'r_leg_hpy'));
          %joint_constraint = joint_constraint.setJointLimits([l_leg_hpx_ind;r_leg_hpx_ind;l_leg_hpy_ind;r_leg_hpy_ind],[obj.l_hpx_lb;-obj.l_hpx_ub;obj.l_hpy_lb;obj.l_hpy_lb],[obj.l_hpx_ub;-obj.l_hpx_lb;obj.l_hpy_ub;obj.l_hpy_ub]);
          joint_constraint = joint_constraint.setJointLimits([l_leg_hpx_ind;r_leg_hpx_ind],[obj.l_hpx_lb;-obj.l_hpx_ub],[obj.l_hpx_ub;-obj.l_hpx_lb]);

          %stance_constraint = Point2PointDistanceConstraint(obj.r,obj.l_foot_body,obj.r_foot_body,[0;0;0],[0;0;0],0.4,inf,[0 1]);
          stance_constraint = Point2PointDistanceConstraint(obj.r,obj.l_foot_body,obj.r_foot_body,l_foot_contact_pts,r_foot_contact_pts,obj.stance_lb*ones(1,size(r_foot_contact_pts,2)),obj.stance_ub*ones(1,size(r_foot_contact_pts,2)),[0 1]);
               
          %============================
          [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
            iktraj_tbreaks,iktraj_qseed_traj,iktraj_qnom_traj,...
            iktraj_rhand_constraint{:},iktraj_lhand_constraint{:},...
            iktraj_rfoot_constraint{:},iktraj_lfoot_constraint{:},...
            iktraj_pelvis_constraint{:},iktraj_head_constraint{:},...
            joint_constraint,stance_constraint,iktraj_dist_constraint{:},qsc,...
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
          qtraj = PPTrajectory(spline(s_breaks,q_breaks));

          s = linspace(0,1,10);
          nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
          xtraj_atlas = zeros(2*nq_atlas,1);
          disp('Publishing candidate endpose for ee_loci ...');
          send_status(3,0,0,'Publishing candidate endpose for ee_loci...');
            
          for j = 1:length(s),
            
            utime = now() * 24 * 60 * 60;
            q_tmp = qtraj.eval(s(j));
            xtraj_atlas(1:nq_atlas) = q_tmp(obj.atlas2robotFrameIndMap(1:nq_atlas));
            obj.pose_pub.publish(xtraj_atlas,utime);
            pause(0.1);
          end
          
           % TODO: Update Cache for Keyframe Adjustment
          
        end
        %-----------------------------------------------------------------------------------------------------------------
        function runPoseOptimizationViaSingleTimeIK(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags)
            
            disp('Generating candidate endpose...');
            send_status(3,0,0,'Generating candidate endpose via IK...');
            
            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
            
            r_foot_pts = [0;0;0];
            l_foot_pts = [0;0;0];
            r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            r_foot_contact_pts = r_foot_contact_pts(:,r_foot_contact_pts(3,:)<=mean(r_foot_contact_pts(3,:))); % only use the bottom ones
            l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            l_foot_contact_pts = l_foot_contact_pts(:,l_foot_contact_pts(3,:)<=mean(l_foot_contact_pts(3,:)));% only use the bottom ones
            num_r_foot_pts = size(r_foot_pts,2);
            num_l_foot_pts = size(l_foot_pts,2);
            
            r_foot_pose0 = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_pts,2);
            r_foot_contact_pos = forwardKin(obj.r,kinsol,obj.r_foot_body,r_foot_contact_pts,0);
            l_foot_pose0 = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_pts,2);
            l_foot_contact_pos = forwardKin(obj.r,kinsol,obj.l_foot_body,l_foot_contact_pts,0);
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            pelvis_pose0 = forwardKin(obj.r,kinsol,obj.pelvis_body,[0;0;0],2);
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
            r_hand_pose0 = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],2);
            l_hand_pose0 = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],2);
            
                
            
            % Solve IK
            timeIndices = unique(Indices);
            N = length(timeIndices);
            if(N>1)
                disp('Error: ERROR optimization expects constraint at a single timestamp. Constraints at multiple times received.');
                send_status(3,0,0,'ERROR: Pose optimization expects constraint at a single timestamp. Constraints at multiple times received.');
                return;
            end
            
            q_guess =q0;
            %l_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
            l_foot_pose0(1:2,:)=nan(2,num_l_foot_pts);
            r_foot_pose0(1:2,:)=nan(2,num_r_foot_pts);
            head_pose0(1:3)=nan(3,1); % only set head orientation not position
            pelvis_pose0(1:2)=nan(2,1); % The problem is to find the pelvis pose
            utorso_pose0(1:2)=nan(2,1);
            r_hand_pose = nan(7,1);
            l_hand_pose = nan(7,1);
            if(goal_type_flags.rh == 2)
                rhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.r_hand_body,obj.rh_gaze_axis,rh_ee_goal(1:3),[0;0;0],obj.hand_gaze_tol)};
            else
                rhand_constraint = {};
            end
            if(goal_type_flags.lh == 2)
                lhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.l_hand_body,obj.lh_gaze_axis,lh_ee_goal(1:3),[0;0;0],obj.hand_gaze_tol)};
            else
                lhand_constraint = {};
            end
            if(goal_type_flags.h == 2)
                head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,h_ee_goal(1:3),[0;0;0],obj.head_gaze_tol)};
            else
                head_constraint = {};
            end
            l_foot_pose = l_foot_pose0;
            r_foot_pose = r_foot_pose0;
  
            lfoot_constraint ={WorldPositionConstraint(obj.r,obj.l_foot_body,l_foot_contact_pts,...
                [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)],...
                [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)])};
            rfoot_constraint = {WorldPositionConstraint(obj.r,obj.r_foot_body,r_foot_contact_pts,...
                [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)],...
                [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)])};
            
            %         foot_origins =[[0;0;0] [0;0;0]]
            %         rel_feet_constraint ={RelativePositionConstraint(obj.r,foot_origins,...
            %             [repmat([-0.25;-0.25],1,size(foot_origins,2));zeros(1,size(foot_origins,2))],...
            %             [repmat([0.25;0.25],1,size(foot_origins,2));zeros(1,size(foot_origins,2))],...
            %             obj.l_foot_body, obj.r_foot_body)};
            

            %head_constraint = [head_constraint,{WorldQuatConstraint(obj.r,obj.head_body,head_pose0(4:7),1e-4)}];
            %       pelvis_constraint = parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose0,0,1e-2,[-inf inf]);
            pelvis_constraint = {};
            %        utorso_constraint = parse2PosQuatConstraint(obj.r,obj.utorso_body,[0;0;0],utorso_pose0,0,1e-2,[-inf inf]);
            
            ik_dist_constraint = {};
            
            pelvis_constraint = {WorldGazeDirConstraint(obj.r,obj.pelvis_body,[0;0;1],[0;0;1],obj.pelvis_upright_gaze_tol)};
            ind=find(Indices==timeIndices(1));
            for k=1:length(ind),
                if(strcmp('pelvis',ee_names{ind(k)}))
                    pelvisT = ee_loci(:,ind(k));
                    pelvis_constraint = {WorldQuatConstraint(obj.r,obj.pelvis_body,rpy2quat(pelvisT(4:6)),1e-2)};
                elseif(strcmp(obj.lh_name,ee_names{ind(k)}))
                    l_ee_goal = ee_loci(:,ind(k));
                    lhandT = zeros(6,1);
                    T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                    T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
                    lhandT(1:3) = T_world_hand_l(1:3,4);
                    lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                    l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                    tspan = [Indices(ind(k)) Indices(ind(k))];
                    lhand_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose,1e-2,1e-4,tspan);
                    obj.plan_cache.lhand_constraint_cell = [obj.plan_cache.lhand_constraint_cell lhand_constraint];
                    dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,tspan),
                                       Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.l_lleg_body,[0;0;0],[0;0;0],obj.ee_lleg_dist_lb,inf,tspan),};
                    ik_dist_constraint = [ik_dist_constraint,dist_constraint];   
                elseif(strcmp(obj.rh_name,ee_names{ind(k)}))
                    r_ee_goal = ee_loci(:,ind(k));
                    rhandT = zeros(6,1);
                    T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                    T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
                    rhandT(1:3) = T_world_hand_r(1:3,4);
                    rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                    r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                    tspan = [Indices(ind(k)) Indices(ind(k))];
                    rhand_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose,1e-2,1e-4,tspan);
                    obj.plan_cache.rhand_constraint_cell = [obj.plan_cache.rhand_constraint_cell rhand_constraint];
                    dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.r_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,tspan),
                                       Point2PointDistanceConstraint(obj.r,obj.r_hand_body,obj.r_lleg_body,[0;0;0],[0;0;0],obj.ee_lleg_dist_lb,inf,tspan),};
                    ik_dist_constraint = [ik_dist_constraint,dist_constraint];  
                elseif (strcmp('l_foot',ee_names{ind(k)}))
                    lfootT = ee_loci(:,ind(k));
                    l_foot_pose = [lfootT(1:3); rpy2quat(lfootT(4:6))];
                    tspan = [Indices(ind(k)) Indices(ind(k))];
                    lfoot_constraint = parse2PosQuatConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose,1e-6,1e-6,tspan);
                elseif(strcmp('r_foot',ee_names{ind(k)}))
                    rfootT = ee_loci(:,ind(k));
                    r_foot_pose = [rfootT(1:3); rpy2quat(rfootT(4:6))];
                    tspan = [Indices(ind(k)) Indices(ind(k))];
                    rfoot_constraint = parse2PosQuatConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose,1e-6,1e-6,tspan);
                else
                    disp('currently only feet/hands and pelvis are allowed');
                end
            end
      
            cost = getCostVector(obj);
            ikoptions = IKoptions(obj.r);
            ikoptions = ikoptions.setQ(diag(cost(1:getNumDOF(obj.r))));
            ikoptions = ikoptions.setDebug(true);
            %ik_qnom = q0;
            ikoptions = ikoptions.setMajorIterationsLimit(1000);
            qsc = QuasiStaticConstraint(obj.r);
            qsc = qsc.setActive(true);
            qsc = qsc.setShrinkFactor(obj.shrinkfactor);
            qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);        
            
            ikoptions = ikoptions.setQ( diag(cost(1:getNumDOF(obj.r))));
            nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
            qstar = nomdata.xstar(1:obj.r.getNumDOF());
            ik_qnom = qstar;            
            %  			ikoptions.q_nom = qstar;
            
            NSamples = 10;
            yaw_angles_bnd = 25;
            arm_loci_flag = ~cellfun(@(x) isempty(strfind(char(x),'palm')),ee_names);
            %arm_loci_flag=(arm_loci_flag)&(Indices==Indices(ind(k)));
            if(sum(arm_loci_flag) == 2)
                nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_two_hands_reaching.mat'));
                qstar = nomdata.xstar(1:obj.r.getNumDOF());
                ik_qnom = qstar;
                tspan = [Indices(ind(k)) Indices(ind(k))];
                rhand_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose,1e-4,sind(2).^2,tspan);
                lhand_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose,1e-4,sind(2).^2,tspan);
                ik_dist_constraint = {};
                dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,tspan),
                                   Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.l_lleg_body,[0;0;0],[0;0;0],obj.ee_lleg_dist_lb,inf,tspan),
                                   Point2PointDistanceConstraint(obj.r,obj.r_hand_body,obj.r_lleg_body,[0;0;0],[0;0;0],obj.ee_lleg_dist_lb,inf,tspan)};
                ik_dist_constraint = [ik_dist_constraint,dist_constraint];  
                NSamples = 20;
                yaw_samples_bnd = 60;
            end
            q_sample = zeros(obj.r.getNumDOF,NSamples);
            sample_cost = zeros(1,NSamples);
            z_bnd = 0.5;
            for k=1:NSamples,
                %q_guess = qstar;
                q_guess(3) = q_guess(3)+2*(rand(1,1)-0.5)*(z_bnd);
                q_guess(6)=q_guess(6)+2*(rand(1,1)-0.5)*(yaw_angles_bnd*pi/180);%+-10degrees from current pose

                
                % if(obj.isBDIManipMode()) % replace feet with pelvis in BDI Manip Mode
                % 
                %     if(~isempty(head_constraint))
                %         [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                %             pelvis_constraint{:},rhand_constraint{:},lhand_constraint{:},head_constraint{:},...
                %             obj.joint_constraint,qsc,ikoptions);
                %     else
                %         [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                %             pelvis_constraint{:},rhand_constraint{:},lhand_constraint{:},...
                %             obj.joint_constraint,qsc,ikoptions);
                %     end
                %  else
                
                joint_constraint = PostureConstraint(obj.r);
                joint_constraint = joint_constraint.setJointLimits((1:obj.r.getNumDOF)',obj.joint_constraint.lb,obj.joint_constraint.ub);
                coords = obj.r.getStateFrame.coordinates;
                % urf limits are lower="-0.523599" upper="0.523599" 

                l_leg_hpx_ind = find(strcmp(coords,'l_leg_hpx'));      r_leg_hpx_ind = find(strcmp(coords,'r_leg_hpx'));
                l_leg_hpy_ind = find(strcmp(coords,'l_leg_hpy'));      r_leg_hpy_ind = find(strcmp(coords,'r_leg_hpy'));
                %joint_constraint = joint_constraint.setJointLimits([l_leg_hpx_ind;r_leg_hpx_ind;l_leg_hpy_ind;r_leg_hpy_ind],[obj.l_hpx_lb;-obj.l_hpx_ub;obj.l_hpy_lb;obj.l_hpy_lb],[obj.l_hpx_ub;-obj.l_hpx_lb;obj.l_hpy_ub;obj.l_hpy_ub]);
                joint_constraint = joint_constraint.setJointLimits([l_leg_hpx_ind;r_leg_hpx_ind],[obj.l_hpx_lb;-obj.l_hpx_ub],[obj.l_hpx_ub;-obj.l_hpx_lb]);

                stance_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_foot_body,obj.r_foot_body,l_foot_contact_pts,r_foot_contact_pts,obj.stance_lb*ones(1,size(r_foot_contact_pts,2)),obj.stance_ub*ones(1,size(r_foot_contact_pts,2)))};
                total_ik_trials = 30;
                if(~isempty(head_constraint))
                    [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKinRepeatSearch(obj.r,total_ik_trials,q_guess,ik_qnom,...
                        rhand_constraint{:},lhand_constraint{:},rfoot_constraint{:},lfoot_constraint{:},head_constraint{:},pelvis_constraint{:},...
                        ik_dist_constraint{:},stance_constraint{:},joint_constraint,qsc,ikoptions);
                else
                    [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKinRepeatSearch(obj.r,total_ik_trials,q_guess,ik_qnom,...
                        rhand_constraint{:},lhand_constraint{:},rfoot_constraint{:},lfoot_constraint{:},pelvis_constraint{:},...
                        ik_dist_constraint{:},stance_constraint{:},joint_constraint,qsc,ikoptions);
                end
                % end
                
                if(snopt_info > 10)
                    warning(['poseOpt IK fails']);
                    send_msg = sprintf('snopt_info = %d. endpose IK fails.',snopt_info);
                    send_status(4,0,0,send_msg);
                    display(infeasibleConstraintMsg(infeasible_constraint));
                end
                if(snopt_info < 10)
                    sample_cost(:,k) = (q_sample(:,k)-ik_qnom)'*ikoptions.Q*(q_sample(:,k)-ik_qnom);
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
            nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
            xtraj_atlas = zeros(2*nq_atlas,1);
            xtraj_atlas(1:nq_atlas,:) = q_out(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
            obj.pose_pub.publish(xtraj_atlas,utime);
            
            %TODO: Update Plan Cache
            s = [0 1];
            q = [q_out q_out];
            obj.plan_cache.s = s;
            obj.plan_cache.s_breaks = s;
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            obj.plan_cache.qsc = obj.plan_cache.qsc.setActive(qsc.active);
            
            % cache foot or pelvis constraints for end poses depending on Hardware Mode
            if(obj.isBDIManipMode())
                obj.plan_cache.pelvis_constraint_cell = [obj.plan_cache.pelvis_constraint_cell pelvis_constraint];
                obj.plan_cache.rfoot_constraint_cell = [obj.plan_cache.rfoot_constraint_cell rfoot_constraint];
                obj.plan_cache.lfoot_constraint_cell = [obj.plan_cache.lfoot_constraint_cell lfoot_constraint];
                obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);
            else
                % TODO: VERIFY
                obj.plan_cache.rfoot_constraint_cell = [obj.plan_cache.rfoot_constraint_cell rfoot_constraint];
                obj.plan_cache.lfoot_constraint_cell = [obj.plan_cache.lfoot_constraint_cell lfoot_constraint];
                obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);
            end
            
            
        end  % end function
        %-----------------------------------------------------------------------------------------------------------------
        function cost = getCostVector(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 1;
            cost.base_y = 1;
            cost.base_z = 0;
            cost.base_roll = 1000;
            cost.base_pitch = 1000;
            cost.base_yaw = 1;
            cost.back_bkz = 10;
            cost.back_bky = 100000;
            cost.back_bkx = 100000;
            cost.neck_ay =  0;
            cost.l_arm_usy = 1;
            cost.l_arm_shx = 1;
            cost.l_arm_ely = 0.5; % encourage elbows and wrist
            cost.l_arm_elx = 0.5;
            cost.l_arm_uwy = 0.25;
            cost.l_arm_mwx = 0.25;
            cost.l_leg_hpz = 1;
            cost.l_leg_hpx = 1;
            cost.l_leg_hpy = 1;
            cost.l_leg_kny = 0.5;
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
        end
        %-----------------------------------------------------------------------------------------------------------------                    
        function pose_quat_constraints = convertEELociFromRPYToQuat(obj,pose_rpy_constraints)
            % input 6xN matrix
            % output7xN matrix
            N= size(pose_rpy_constraints,2);
            pose_quat_constraints=zeros(7,N);
            pose_quat_constraints(1:3,:)=pose_rpy_constraints(1:3,:);
            for k=1:N,
                temp=rpy2quat(pose_rpy_constraints(4:6,k));
                pose_quat_constraints(4:7,k)=temp(:);
            end
        end
        %-----------------------------------------------------------------------------------------------------------------
                   
    end% end methods
end% end classdef
