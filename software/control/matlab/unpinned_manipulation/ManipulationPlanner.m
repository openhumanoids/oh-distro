classdef ManipulationPlanner < handle
    
    properties
        num_breaks
        s_breaks
        q_breaks
        qdos_breaks
        plan_pub
        map_pub
        pose_pub
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
        v_desired
        r_hand_body
        l_hand_body
        head_body
        l_foot_body
        r_foot_body
        pelvis_body
        utorso_body
        
        head_gaze_target
        lhand_gaze_target
        rhand_gaze_target

        planning_mode % 1 if ik sequence is on, 2 if use IK only, 3 if use teleop
    end
    
    methods
        function obj = ManipulationPlanner(r)
            obj.r = r;
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.num_breaks = 4;
            obj.v_desired = 0.1; % 10cm/sec seconds, hard coded for now
            %obj.plan_pub = RobotPlanPublisherWKeyFrames('atlas',joint_names,true,'CANDIDATE_MANIP_PLAN',obj.num_breaks);
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
            obj.map_pub = AffIndexedRobotPlanPublisher('CANDIDATE_MANIP_MAP',true,joint_names);
            obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
            obj.restrict_feet=true;
            obj.planning_mode = 1;
            obj.r_hand_body = findLink(obj.r,'r_hand');
            obj.l_hand_body = findLink(obj.r,'l_hand');
            obj.r_foot_body = obj.r.findLink('r_foot');
            obj.l_foot_body = obj.r.findLink('l_foot');
            obj.head_body = obj.r.findLink('head');
            obj.pelvis_body = findLink(obj.r,'pelvis');
            obj.utorso_body = findLink(obj.r,'utorso');
        end

        function setPlanningMode(obj,val)
              obj.planning_mode  = val;
        end


        function adjustAndPublishManipulationPlan(obj,x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint,goal_type_flags)
            is_keyframe_constraint = true;
            runOptimization(obj,x0,rh_ee_constraint,lh_ee_constraint,rf_ee_constraint,lf_ee_constraint,h_ee_constraint,goal_type_flags,is_keyframe_constraint,[]);
        end
        
        function generateAndPublishManipulationPlan(obj,varargin)
            
            switch nargin
                case 8
                    is_keyframe_constraint = false;
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags =varargin{7}; 
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,goal_type_flags,is_keyframe_constraint);
               case 9
                    is_keyframe_constraint = false;
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags =varargin{7}; 
                    q_desired = varargin{8};
                    runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,goal_type_flags,is_keyframe_constraint,q_desired);
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
        
        function generateAndPublishPosturePlan(obj,x0,q_desired,useIK_state)
            runOptimizationForPosturePlan(obj,x0,q_desired,useIK_state);
        end
        
        function generateAndPublishCandidateRobotEndPose(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint) %#ok<INUSD>
            runPoseOptimization(obj,x0,ee_names,ee_loci,timeIndices);
        end
        
        function runPoseOptimization(obj,x0,ee_names,ee_loci,Indices)

            disp('Generating candidate endpose...');
            send_status(3,0,0,'Generating candidate endpose...');

            q0 = x0(1:getNumDOF(obj.r));
            
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get current hand and foot positions
            kinsol = doKinematics(obj.r,q0);
                        
             
            r_foot_pts = obj.r_foot_body.getContactPoints();
            l_foot_pts = obj.l_foot_body.getContactPoints();
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
            T_hand_palm_l = HT([0;0.1;0],1.57079,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],-1.57079,0,-1.57079);
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

            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_comfortable_right_arm_manip.mat'));
            qstar = nomdata.xstar(1:obj.r.getNumDOF());
          NSamples = 10;
          for k=1:NSamples,
             %q_guess = qstar;
             q_guess(3)=q_guess(3)+2*(rand(1,1)-0.5)*(0.2);% +-20cm
             q_guess(6)=q_guess(6)+2*(rand(1,1)-0.5)*(25*pi/180);%+-25degrees from current pose
             ikoptions.q_nom = q_guess;
				
            ikoptions.quasiStaticFlag = true;
               %obj.pelvis_body,[0;0;0],pelvis_const,...
               %   obj.head_body,[0;0;0],head_pose0_relaxed,...
	               %   obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...
              [q_sample(:,k),snopt_info] = inverseKin(obj.r,q_guess,...
                obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,... 
                obj.r_hand_body,[0;0;0],rhand_const,...
                obj.l_hand_body,[0;0;0],lhand_const,...
                ikoptions);

	             if(snopt_info > 10)
		          warning(['poseOpt IK fails']);
		          send_status(3,0,0,sprintf('snopt_info = %d...',snopt_info));
	             end
	          if(snopt_info < 10)
	             sample_cost(:,k) = (q_sample(:,k)-ikoptions.q_nom)'*ikoptions.Q*(q_sample(:,k)-ikoptions.q_nom);
	          else
               send_status(3,0,0,'Bad candidate startpose...');
	             sample_cost(:,k) =  Inf;
	          end
	          disp(['sample_cost(:,k): ' num2str(sample_cost(:,k))]);
           end
           [~,k_best] = min(sample_cost);
           disp(['sample_cost(:,k): ' num2str(sample_cost(:,k_best))]);
           q_out = q_sample(:,k_best); % take the least cost pose

           % publish robot pose
           disp('Publishing candidate endpose ...');
           kinsol_out = doKinematics(obj.r,q_out);
           [~,J_rh] = forwardKin(obj.r,kinsol_out,obj.r_hand_body,[0;0;0],1);
           fprintf('The condition number of Jacobian matrix of the right hand is %10.4f\n',cond(J_rh));
           send_status(3,0,0,'Publishing candidate endpose...');
           utime = now() * 24 * 60 * 60;
           xtraj = zeros(getNumStates(obj.r),1);
           xtraj(1:getNumDOF(obj.r),:) = q_out;
           obj.pose_pub.publish(xtraj,utime);
        end 
         
        function runOptimizationForPosturePlan(obj,x0,q_desired,useIK_state)
            disp('Generating posture plan...');
            q0 = x0(1:getNumDOF(obj.r));
            s = [0 1];
            nq = obj.r.getNumDOF();
            
            kinsol = doKinematics(obj.r,q_desired);
            
            %             rf_ee_goal = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],1);
            %             lf_ee_goal = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],1);
            %             rh_ee_goal = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
            %             lh_ee_goal = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);
            %             h_ee_goal  = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],1);
            %             is_keyframe_constraint = false;
            %             runOptimization(obj,x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,is_keyframe_constraint,q_desired);
            
            obj.rfootT = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],1);
            obj.lfootT = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],1);
            obj.rhandT = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
            obj.lhandT = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);
            obj.headT  = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],1);
            if(useIK_state == 1)
              kinsol0 = doKinematics(obj.r,q0);
              rfoot0 = forwardKin(obj.r,kinsol0,obj.r_foot_body,[0;0;0],1);
              lfoot0 = forwardKin(obj.r,kinsol0,obj.l_foot_body,[0;0;0],1);
              ikargs = {obj.r_foot_body,[0;0;0],rfoot0,obj.l_foot_body,[0;0;0],lfoot0,...
                obj.r_hand_body,[0;0;0],obj.rhandT,obj.l_hand_body,[0;0;0],obj.lhandT,...
                obj.head_body,[0;0;0],obj.headT};
              cost = diag(obj.getCostVector);
              cost = cost(1:nq,1:nq);
              [q_desired,info] = inverseKin(obj.r,q_desired,ikargs{:},struct('Q',cost));
              if(info>10)
                fprintf('IK info = %d in posture plan optimization\n',info);
              end
            elseif(useIK_state ==2) % Foot in contact
              ikoptions.jointLimitMax = [inf(6,1);q_desired(7:end)];
              ikoptions.jointLimitMin = [-inf(6,1);q_desired(7:end)];
              lfoot_const = struct();
              kinsol0 = doKinematics(obj.r,q0);
              rfoot0 = forwardKin(obj.r,kinsol0,obj.r_foot_body,[0;0;0],1);
              lfoot0 = forwardKin(obj.r,kinsol0,obj.l_foot_body,[0;0;0],1);
              ikargs = {obj.r_foot_body,[0;0;0],rfoot0,obj.l_foot_body,[0;0;0],lfoot0};
              ikoptions.q_nom = q0;
              [q_desired,info] = inverseKin(obj.r,q0,ikargs{:},ikoptions);
              if(info>10)
                fprintf('IK info = %d in posture plan optimization\n',info);
              end
            end
            qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_desired]));
            s = linspace(0,1,4);
            s_breaks = linspace(s(1),s(end),obj.num_breaks);
            obj.s_breaks = s_breaks;           
            s = unique([s(:);s_breaks(:)]);
            q = zeros(length(q0),length(s));
            for i=1:length(s),
                q(:,i) = qtraj_guess.eval(s(i));
            end
            obj.qtraj_guess_fine = PPTrajectory(spline(s, q));
            disp('Publishing posture plan...');
            xtraj = zeros(getNumStates(obj.r)+2,length(s));
            xtraj(1,:) = 0*s;
            
            
            % calculate end effectors breaks via FK.
            q_break = zeros(nq,length(s_breaks));
            rhand_breaks = zeros(7,length(s_breaks));
            lhand_breaks = zeros(7,length(s_breaks));
            head_breaks = zeros(7,length(s_breaks));
            rfoot_breaks = zeros(7,length(s_breaks));
            lfoot_breaks = zeros(7,length(s_breaks));
            for brk =1:length(s_breaks),
                q_break(:,brk) = obj.qtraj_guess_fine.eval(s_breaks(brk));
                kinsol_tmp = doKinematics(obj.r,q_break(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
%                 if(brk>1)
%                   ikargs_break = {obj.r_hand_body,[0;0;0],rhand_breaks(:,brk),obj.l_hand_body,[0;0;0],lhand_breaks(:,brk),...
%                     obj.r_foot_body,[0;0;0],obj.rfootT,obj.l_foot_body,[0;0;0],obj.lfootT,obj.head_body,[0;0;0],head_breaks(:,brk)};
%                   cost = diag(obj.getCostVector());
%                   cost = cost(1:nq,1:nq);
%                   q_break(:,brk) = inverseKin(obj.r,q_break(:,brk-1),ikargs_break{:},struct('Q',cost));
%                 end
            end
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head);
%             s_total = max(max(s_total_lh,s_total_rh),s_total_head);
            s_total = max(s_total,0.01);
            
            for l =1:length(s_breaks),
                ind = find(abs(s - s_breaks(l))<1e-3);
                xtraj(1,ind) = 1.0;
                xtraj(2,ind) = 0.0;
            end
            xtraj(2+(1:nq),:) = q_break;
            
            ts = s.*(s_total/obj.v_desired); % plan timesteps
            obj.time_2_index_scale = (obj.v_desired/s_total);
            
            %obj.plan_pub.publish(ts,xtraj);
            utime = now() * 24 * 60 * 60;
            
             % ignore the first state
             % ts = ts(2:end);
             % xtraj=xtraj(:,2:end);
            obj.plan_pub.publish(xtraj,ts,utime);
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
            
            r_foot_pts = obj.r_foot_body.getContactPoints();
            l_foot_pts = obj.l_foot_body.getContactPoints();
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
            utorso_pose0 = forwardKin(obj.r,kinsol,obj.utorso_body,[0;0;0],2);
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
            T_palm_grasp = HT([0.05;0;0],0,0,0); % We evaluate the achievement of hand grasps based upon a notional grasp point
            T_grasp_palm = inv_HT(T_palm_grasp);
            
            ind = getActuatedJoints(obj.r);
            cost = getCostVector(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.MajorIterationsLimit = 100;

						%%% Hongkai added - to prevent the robot from leaning back - when getting manip-maps (and plans possibly)
						coords = obj.r.getStateFrame();
						[joint_min,joint_max] = obj.r.getJointLimits();
						joint_min = Point(coords,[joint_min;0*joint_min]);
						joint_min.back_mby = -.2;
						joint_min = double(joint_min);
						ikoptions.jointLimitMin = joint_min(1:obj.r.getNumDOF());            
           
            %joint_max
            
            %Setting a max joint limit on the back also 
            
            joint_max = Point(coords,[joint_max;0*joint_max]);
						joint_max.back_mby = 0.2;
						joint_max = double(joint_max);
						ikoptions.jointLimitMax = joint_max(1:obj.r.getNumDOF());
            
						%% Might be worth preventing leaning forward too much also 
            
            if(~is_manip_map)
                obj.rfootT = forwardKin(obj.r,kinsol,obj.r_foot_body,[0;0;0],1);
                obj.lfootT = forwardKin(obj.r,kinsol,obj.l_foot_body,[0;0;0],1);
                obj.rhandT = forwardKin(obj.r,kinsol,obj.r_hand_body,[0;0;0],1);
                obj.lhandT = forwardKin(obj.r,kinsol,obj.l_hand_body,[0;0;0],1);
                obj.headT  = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],1);
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
                  lhand_const.gaze_axis = [1;0;0];
                  lhand_const.gaze_conethreshold = pi/12;
                end
                %r_hand_pose0= [nan;nan;nan;nan;nan;nan;nan];
                if(goal_type_flags.rh ~=2)
                  rhand_const.min = r_hand_pose0-1e-2*[ones(3,1);ones(4,1)];
                  rhand_const.max = r_hand_pose0+1e-2*[ones(3,1);ones(4,1)];
                else
                  rhand_const.type = 'gaze';
                  rhand_const.gaze_target = ee_loci(1:3,i);
                  rhand_const.gaze_axis = [1;0;0];
                  rhand_const.gaze_conethreshold = pi/12;
                end
                if(goal_type_flags.h ==2)
                  head_const.type = 'gaze';
                  head_const.gaze_target = ee_loci(1:3,i);
                  head_const.gaze_axis = [1;0;0];
                  head_const.gaze_conethreshold = pi/12;
                end
                %l_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
%                 lfoot_const.min = l_foot_pose0-1e-2*[ones(3,1);ones(4,1)];
%                 lfoot_const.max = l_foot_pose0+1e-2*[ones(3,1);ones(4,1)]; 
                %r_foot_pose0= [nan;nan;nan;nan;nan;nan;nan];
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
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('l_hand')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_name;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indices(i).dof_value;Indices(ind(k)).dof_value(1)];
                            %plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose l_hand_pose];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose l_grasp_pose];
                        else
                            if(i==length(timeIndices))
                                obj.lhandT = l_ee_goal;
                            end
                        end
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
                        if(is_manip_map)
                            plan_Indices(i).aff_type=Indices(ind(k)).aff_type;
                            plan_Indices(i).aff_uid=Indices(ind(k)).aff_uid;
                            plan_Indices(i).num_ees=plan_Indices(i).num_ees+1;
                            plan_Indices(i).ee_name=[plan_Indices(i).ee_name;java.lang.String('r_hand')];
                            plan_Indices(i).dof_name=[plan_Indices(i).dof_name;Indices(ind(k)).dof_name(1)];
                            plan_Indices(i).dof_value=[plan_Indices(i).dof_value;Indices(ind(k)).dof_value(1)];
                            %plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose r_hand_pose];
                            plan_Indices(i).dof_pose=[plan_Indices(i).dof_pose r_grasp_pose];
                        else
                            if(i==length(timeIndices))
                                obj.rhandT = r_ee_goal;
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
                                obj.lfootT = l_foot_pose;
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
                                obj.rfootT = r_foot_pose;
                            end
                        end
                    end
                    
                end
                
                ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
                ikoptions.q_nom = q_guess;
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
                     %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                     %obj.head_body,[0;0;0],head_pose0_relaxed,...
                     % Hongkai to investigate this.$$$$#!%%%%#$#%#$%^%$^%$^%$^%$6546546 HONGKAI  
                    [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                        obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                        obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                        obj.r_hand_body,[0;0;0],rhand_const, ...
                        obj.l_hand_body,[0;0;0],lhand_const,...
                        obj.head_body,[0;0;0],head_const,...
                        ikoptions);
                end
                
                q_guess =q(:,i);
                toc;
                if(snopt_info > 10)
                    warning(['The IK fails at ',num2str(i)]);
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
                        %map_pose = T_palm_grasp * T_hand_palm_l * map_pose;
                        T_world_m_grasp = T_world_m_hand * T_hand_palm_l * T_palm_grasp;
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
                            [temp,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(l_hand_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        elseif (strcmp('r_hand',ee_temp(i).ee_name))
                            [temp,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(r_hand_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2));
                        elseif (strcmp('l_foot',ee_temp(i).ee_name))
                            [temp,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(l_foot_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2)); 
                        elseif (strcmp('r_foot',ee_temp(i).ee_name))
                            [temp,jcur] = min(sum((ee_temp(i).map_pose(1:3,:)-repmat(r_foot_pose0(1:3),1,size(ee_temp(i).map_pose,2))).^2)); 
                        end;
                        
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
                keyframe_inds = unique(round(linspace(1,length(timeIndices),obj.num_breaks))); % no more than ${obj.num_breaks} keyframes
                xtraj(1,keyframe_inds) = 1.0;
                xtraj(2,:) = 0*timeIndices;
                xtraj(3:getNumDOF(obj.r)+2,:) = q;
                
                s = (timeIndices-min(timeIndices))/(max(timeIndices)-min(timeIndices));
                
                %timeIndices
                
                fprintf('Max : %f - Min : %f', max(timeIndices), min(timeIndices));
                    
                %s
                
                %s
                
                obj.qtraj_guess_fine = PPTrajectory(spline(s, q));
                
                s_sorted = sort(s);
                s_breaks = s_sorted(keyframe_inds);
                obj.s_breaks = s_breaks;
                % calculate end effectors breaks via FK.
                for brk =1:length(s_breaks),
                    q_break = obj.qtraj_guess_fine.eval(s_breaks(brk));
                    kinsol_tmp = doKinematics(obj.r,q_break);
                    rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                    lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                    rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                    lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                    head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
                end
                s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
                s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
                s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
                s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
                s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
                s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head); 
                s_total = max(s_total,0.01);
                
                ts = s.*(s_total/obj.v_desired); % plan timesteps
                obj.time_2_index_scale = (obj.v_desired/s_total);
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
                            G(cnt).utime =  s_transition.*(s_total/obj.v_desired);
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
                            G(cnt).utime =  s_transition.*(s_total/obj.v_desired);
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
                
            end
        end

        function runOptimization(obj,varargin)
            is_locii = false;
            is_keyframe_constraint = false;
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
                case 9
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags= varargin{7};
                    is_keyframe_constraint = varargin{8};
                case 10
                    x0 = varargin{1};
                    rh_ee_goal= varargin{2};
                    lh_ee_goal= varargin{3};
                    rf_ee_goal= varargin{4};
                    lf_ee_goal= varargin{5};
                    h_ee_goal = varargin{6};
                    goal_type_flags= varargin{7};
                    is_keyframe_constraint = varargin{8};
                    q_desired = varargin{9};
                case 6
                    x0 = varargin{1};
                    ee_names= varargin{2};
                    ee_loci = varargin{3};
                    timeIndices = varargin{4};
                    postureconstraint =varargin{5};
                    % joint_timestamps=[postureconstraint.time];
                    % joint_names = {postureconstraint.name};
                    % joint_positions = [postureconstraint.joint_position];
                    is_locii = true;
              otherwise
                    error('Incorrect usage of runOptimization in Manip Planner. Undefined number of vargin.')
            end
            
            
            disp('Generating plan...');
            send_status(3,0,0,'Generating  plan...');
            
            q0 = x0(1:getNumDOF(obj.r));
            T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
            
            % get foot positions
            kinsol = doKinematics(obj.r,q0);
            r_foot_pts = obj.r_foot_body.getContactPoints();
            l_foot_pts = obj.l_foot_body.getContactPoints();
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
            
            % Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            T_hand_palm_l = HT([0;0.1;0],1.57079,0,1.57079);
            T_palm_hand_l = inv_HT(T_hand_palm_l);
            T_hand_palm_r = HT([0;-0.1;0],-1.57079,0,-1.57079);
            T_palm_hand_r = inv_HT(T_hand_palm_r);
            
            
            % Get head position
            
            head_pose0 = forwardKin(obj.r,kinsol,obj.head_body,[0;0;0],2);
            %======================================================================================================
            
            if(~is_keyframe_constraint)
                
                
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
                    T_world_hand_r = T_world_palm_r*T_palm_hand_r;
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
                      T_world_hand_l = T_world_palm_l*T_palm_hand_l;
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
                              
                if(goal_type_flags.h == 2)
                % headT(1:3) is actually object pos
                 obj.head_gaze_target = headT(1:3);
                else
                  obj.head_gaze_target = [];
                end
                if(goal_type_flags.lh == 2)
                 % lhandT(1:3) is actually object pos
                 obj.lhand_gaze_target = lhandT(1:3);
                else
                  obj.lhand_gaze_target = [];
                end   
                if(goal_type_flags.rh == 2)
                  % rhandT(1:3) is actually object pos
                 obj.rhand_gaze_target = rhandT(1:3);
                else
                  obj.rhand_gaze_target = [];
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
                obj.rhandT = rhandT;
                obj.lhandT = lhandT;
                obj.rfootT = rfootT;
                obj.lfootT = lfootT;
                obj.headT = headT;
                
            elseif((is_keyframe_constraint)&&(~is_locii))
                
                rhandT =  obj.rhandT;
                lhandT =  obj.lhandT;
                if(size(obj.rfootT,2)~= num_r_foot_pts)
                  T_world_foot_r = HT(obj.rfootT(1:3),obj.rfootT(4),obj.rfootT(5),obj.rfootT(6));
                  rfootT = T_world_foot_r*[r_foot_pts;ones(1,num_r_foot_pts)];
                  rfootT = [rfootT(1:3,:);repmat(obj.rfootT(4:6),1,num_r_foot_pts)];
                else
                  rfootT = obj.rfootT;
                end
                if(size(obj.lfootT,2)~= num_l_foot_pts)
                  T_world_foot_l = HT(obj.lfootT(1:3),obj.lfootT(4),obj.lfootT(5),obj.lfootT(6));
                  lfootT = T_world_foot_l*[r_foot_pts;ones(1,num_l_foot_pts)];
                  lfootT = [lfootT(1:3,:);repmat(obj.lfootT(4:6),1,num_l_foot_pts)];
                else
                  lfootT = obj.lfootT;
                end
                headT  =  obj.headT;
                r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
                l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];
                r_foot_poseT = [rfootT(1:3,:); repmat(rpy2quat(rfootT(4:6,1)),1,num_r_foot_pts)];
                l_foot_poseT = [lfootT(1:3,:); repmat(rpy2quat(lfootT(4:6,1)),1,num_l_foot_pts)];
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
                    rfoot_int_constraint = nan(6,num_r_foot_pts);
                else
                    rfoot_int_constraint = zeros(6,num_r_foot_pts);
                    s_int_rf = rf_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of palm in world frame
                    rpy = quat2rpy(rf_ee_goal.desired_pose(4:7,1));
                    for k = 1:num_r_foot_pts
                        T_world_foot_r = HT(rf_ee_goal.desired_pose(1:3,k),rpy(1),rpy(2),rpy(3));
                        rfoot_int_constraint(1:3,k) = T_world_foot_r(1:3,4);
                        rfoot_int_constraint(4:6,k) =rotmat2rpy(T_world_foot_r(1:3,1:3));
                    end
                    if(abs(1-s_int_rf)<1e-3)
                        disp('rf end state is modified')
                        for k = 1:num_r_foot_pts
                            r_foot_poseT(1:3,k) = rfoot_int_constraint(1:3,k);
                            r_foot_poseT(4:7,k) = rpy2quat(rfoot_int_constraint(4:6,k));
                            obj.rfootT(:,k) = rfoot_int_constraint(:,k);
                        end
                        rfoot_int_constraint = nan(6,num_r_foot_pts);
                        rf_ee_goal=[];
                    end
                end
                
                if(isempty(lf_ee_goal))
                    s_int_lf= nan;
                    lfoot_int_constraint = nan(6,num_l_foot_pts);
                else
                    lfoot_int_constraint = zeros(6,num_l_foot_pts);
                    s_int_lf = lf_ee_goal.time*obj.time_2_index_scale;
                    % Desired position of left foot in world frame
                    rpy = quat2rpy(lf_ee_goal.desired_pose(4:7,1));
                    for k = 1:num_l_foot_pts
                        T_world_foot_l = HT(lf_ee_goal.desired_pose(1:3,k),rpy(1),rpy(2),rpy(3));
                        lfoot_int_constraint(1:3,k) = T_world_foot_l(1:3,4);
                        lfoot_int_constraint(4:6,k) =rotmat2rpy(T_world_foot_l(1:3,1:3));
                    end
                    if(abs(1-s_int_lf)<1e-3)
                        disp('lf end state is modified')
                        for k = 1:num_l_foot_pts
                            l_foot_poseT(1:3,k) = lfoot_int_constraint(1:3,k);
                            l_foot_poseT(4:7,k) = rpy2quat(lfoot_int_constraint(4:6,k));
                            obj.lfootT(:,k) = lfoot_int_constraint(:,k);
                        end
                        lfoot_int_constraint = nan(6,num_l_foot_pts);
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
                r_foot_pose_int = [rfoot_int_constraint(1:3,:); repmat(rpy2quat(rfoot_int_constraint(4:6,1)),1,num_r_foot_pts)];
                l_foot_pose_int = [lfoot_int_constraint(1:3,:); repmat(rpy2quat(lfoot_int_constraint(4:6,1)),1,num_l_foot_pts)];
                head_pose_int   = [head_int_constraint(1:3); rpy2quat(head_int_constraint(4:6))];
            end
            
            %======================================================================================================
            
            
            cost = getCostVector(obj);
            
            ikoptions = struct();
            ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
            ikoptions.q_nom = q0;
            ikoptions.quasiStaticFlag = true;
            ikoptions.shrinkFactor = 0.95;
            if(is_keyframe_constraint)
                ikoptions.MajorIterationsLimit = 300;
            else
                ikoptions.MajorIterationsLimit = 500;
            end
            
            %%%%%%%%%%%%%%%%%%%% Setting the Torso max and min 
            coords = obj.r.getStateFrame();
            [joint_min,joint_max] = obj.r.getJointLimits();
            joint_min = Point(coords,[joint_min;0*joint_min]);
            joint_min.back_mby = -.2;
            joint_min = double(joint_min);
            ikoptions.jointLimitMin = joint_min(1:obj.r.getNumDOF());
           
            %joint_max
            
            %Setting a max joint limit on the back also 
            
            joint_max = Point(coords,[joint_max;0*joint_max]);
            joint_max.back_mby = 0.2;
            joint_max = double(joint_max);
            ikoptions.jointLimitMax = joint_max(1:obj.r.getNumDOF());
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
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
            
            
            if(~is_locii) % End State Constraints
                % Constraints for feet
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
                
            else % Motion Constraints
                
                %add eelocii constraints
                t_ind = unique([timeIndices]);
                s_ind = (t_ind  - min(t_ind))./max(t_ind);
                
                if(obj.restrict_feet)
                    kc_rfoot = ActionKinematicConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose0,[s(1),s(end)],'rfoot0');
                    ks = ks.addKinematicConstraint(kc_rfoot);
                    kc_lfoot = ActionKinematicConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose0,[s(1),s(end)],'lfoot0');
                    ks = ks.addKinematicConstraint(kc_lfoot);
                end
                % resolution of constraints is a function of length of (t_ind)
                for i=unique(round(linspace(1,length(t_ind),round(length(t_ind)/3)))),%1:length(t_ind),
                    ind=find(timeIndices==t_ind(i));
                    % Find all active constraints at current index
                    for k=1:length(ind),
                        
                        if(strcmp('left_palm',ee_names{ind(k)}))
                            l_ee_goal = ee_loci(:,ind(k));
                            lhand_at_ind = zeros(6,1);
                            T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
                            T_world_hand_l = T_world_palm_l*T_palm_hand_l;
                            lhand_at_ind(1:3) = T_world_hand_l(1:3,4);
                            lhand_at_ind(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
                            l_hand_pose = [lhand_at_ind(1:3); rpy2quat(lhand_at_ind(4:6))];
                            lhand_const.min = l_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                            lhand_const.max = l_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                            kc_lhand_at_ind = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],lhand_const,[s_ind(i),s_ind(i)],['lhand_at_' num2str(t_ind(i))]);
                            ks = ks.addKinematicConstraint(kc_lhand_at_ind);
                            if(t_ind(i)==max(t_ind))
                                l_hand_poseT = l_hand_pose;
                                obj.lhandT = l_ee_goal;
                            elseif(t_ind(i)==min(t_ind))
                                l_hand_pose0 = l_hand_pose;
                            end
                        end
                        
                        if(strcmp('right_palm',ee_names{ind(k)}))
                            r_ee_goal = ee_loci(:,ind(k));
                            rhand_at_ind = zeros(6,1);
                            T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
                            T_world_hand_r = T_world_palm_r*T_palm_hand_r;
                            rhand_at_ind(1:3) = T_world_hand_r(1:3,4);
                            rhand_at_ind(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
                            r_hand_pose = [rhand_at_ind(1:3); rpy2quat(rhand_at_ind(4:6))];
                            rhand_const.min = r_hand_pose-1e-4*[ones(3,1);ones(4,1)];
                            rhand_const.max = r_hand_pose+1e-4*[ones(3,1);ones(4,1)];
                            kc_rhand_at_ind = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],rhand_const,[s_ind(i),s_ind(i)],['rhand_at_' num2str(t_ind(i))]);
                            ks = ks.addKinematicConstraint(kc_rhand_at_ind);
                            if(t_ind(i)==max(t_ind))
                                r_hand_poseT = r_hand_pose;
                                obj.rhandT = r_ee_goal;
                            elseif(t_ind(i)==min(t_ind))
                                r_hand_pose0 = r_hand_pose;
                            end
                        end
                        
                        if(strcmp('l_foot',ee_names{ind(k)}))
                            lfoot_at_ind = ee_loci(:,ind(k));
                            l_foot_pose = [lfoot_at_ind(1:3); rpy2quat(lfoot_at_ind(4:6))];
                            lfoot_const.min = l_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                            lfoot_const.max = l_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                            kc_lfoot_at_ind = ActionKinematicConstraint(obj.r,obj.l_foot_body,[0;0;0],lfoot_const,[s_ind(i),s_ind(i)],['lfoot_at_' num2str(t_ind(i))]);
                            ks = ks.addKinematicConstraint(kc_lfoot_at_ind);
                            if(t_ind(i)==max(t_ind))
                                l_foot_poseT = l_foot_pose;
                                obj.lfootT = lfoot_at_ind;
                            elseif(t_ind(i)==min(t_ind))
                                l_foot_pose0 = l_foot_pose;
                            end
                        end
                        
                        if(strcmp('r_foot',ee_names{ind(k)}))
                            rfoot_at_ind = ee_loci(:,ind(k));
                            r_foot_pose = [rfootT(1:3); rpy2quat(rfoot_at_ind(4:6))];
                            rfoot_const.min = r_foot_pose-1e-6*[ones(3,1);ones(4,1)];
                            rfoot_const.max = r_foot_pose+1e-6*[ones(3,1);ones(4,1)];
                            kc_rfoot_at_ind = ActionKinematicConstraint(obj.r,obj.r_foot_body,[0;0;0],rfoot_const,[s_ind(i),s_ind(i)],['rfoot_at_' num2str(t_ind(i))]);
                            ks = ks.addKinematicConstraint(kc_rfoot_at_ind);
                            if(t_ind(i)==max(t_ind))
                                r_foot_poseT = r_foot_pose;
                                obj.rfootT = rfoot_at_ind;
                            elseif(t_ind(i)==min(t_ind))
                                r_foot_pose0 = r_foot_pose;
                            end
                        end
                        
                    end % end for
                end % end for
                %============================
                % find the start location (Assuming we are not current posture =! start of manip plan)
                % obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...
                r_foot_pose0_static_contact = struct('max',r_foot_pose0,...
                    'min',r_foot_pose0, ...
                    'contact_state',{ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)});
                l_foot_pose0_static_contact = struct('max',l_foot_pose0,...
                    'min',l_foot_pose0, ...
                    'contact_state',{ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)});
                %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                [q0,snopt_info] = inverseKin(obj.r,q0,...
                    obj.r_foot_body,r_foot_pts,r_foot_pose0_static_contact,...
                    obj.l_foot_body,l_foot_pts,l_foot_pose0_static_contact, ...
                    obj.r_hand_body,[0;0;0],r_hand_pose0, ...
                    obj.l_hand_body,[0;0;0],l_hand_pose0,...
                    ikoptions);
%                     obj.head_body,[0;0;0],head_pose0,...
%                     ikoptions);
                %============================
                
            end
            
            % kc_pelvis = ActionKinematicConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose0,[s(1),s(end)],'pelvis');
            % ks = ks.addKinematicConstraint(kc_pelvis);
            % kc_torso = ActionKinematicConstraint(obj.r,obj.utorso_body,[0;0;0],utorso_pose0_relaxed,[s(1),s(end)],'utorso');
            % ks = ks.addKinematicConstraint(kc_torso);
            
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
                    kc_rhand_intermediate = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose_int,[s_int_rh,s_int_rh],'rhand_int');
                    ks = ks.addKinematicConstraint(kc_rhand_intermediate);
                end
                %if((~isempty(lh_ee_goal))&&(abs(1-s_int_lh)>1e-3))
                if(~isempty(lh_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_lh));
                    s_int_lh=obj.s_breaks(ind);
                    kc_lhand_intermediate = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose_int,[s_int_lh,s_int_lh],'lhand_int');
                    ks = ks.addKinematicConstraint(kc_lhand_intermediate);
                end
                %if((~isempty(rf_ee_goal))&&(abs(1-s_int_rf)>1e-3))
                if(~isempty(rf_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_rf)); % snap to closest break point (avoiding very close double constraints)
                    s_int_rf=obj.s_breaks(ind);
                    kc_rfoot_intermediate = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,r_foot_pose_int,[s_int_rf,s_int_rf],'rfoot_int',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_rfoot_intermediate);
                end
                %if((~isempty(lf_ee_goal))&&(abs(1-s_int_lf)>1e-3))
                if(~isempty(lf_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_lf));
                    s_int_lf=obj.s_breaks(ind);
                    kc_lfoot_intermediate = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,l_foot_pose_int,[s_int_lf,s_int_lf],'lfoot_int',...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                        {ContactAffordance()},...
                        {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                    ks = ks.addKinematicConstraint(kc_lfoot_intermediate);
                end
                if(~isempty(h_ee_goal))
                    [~,ind] = min(abs(obj.s_breaks-s_int_head));
                    s_int_head=obj.s_breaks(ind);
                    kc_head_intermediate = ActionKinematicConstraint(obj.r,obj.head_body,[0;0;0],head_pose_int,[s_int_head,s_int_head],'head_int');
%                     ks = ks.addKinematicConstraint(kc_head_intermediate);
                end
                
            end %if(is_keyframe_constraint)
            
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
                if(~isempty(obj.head_gaze_target))
                  head_const.type = 'gaze';
                  head_const.gaze_target = obj.head_gaze_target;
                  head_const.gaze_conethreshold = pi/12;
                  head_const.gaze_axis = [1;0;0];
                end
                if(~isempty(obj.rhand_gaze_target))
                  rhand_const.type = 'gaze';
                  rhand_const.gaze_target = obj.rhand_gaze_target;
                  rhand_const.gaze_conethreshold = pi/12;
                  rhand_const.gaze_axis = [1;0;0];
                end
                if(~isempty(obj.lhand_gaze_target))
                  lhand_const.type = 'gaze';
                  lhand_const.gaze_target = obj.lhand_gaze_target;
                  lhand_const.gaze_conethreshold = pi/12;
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
                    if(obj.planning_mode == 3)
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
                            rhand_const.max = rhand_pose;
                            rhand_const.min = rhand_pose;
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
                            lhand_const.max = lhand_pose;
                            lhand_const.min = lhand_pose;
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
                          %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        [q_final_guess,snopt_info] = inverseKin(obj.r,q_start,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact, ...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact, ...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            obj.head_body,[0;0;0],head_const,...
                            ikoptions);
                    end
                    
                    if(snopt_info >10)
                        warning('The IK fails at the end');
                        send_status(3,0,0,sprintf('snopt_info = %d. Manip plan initial IK is not very good.',snopt_info));
                    end
                else
                    q_final_guess =q_desired;
                end
                %============================
                
                s_breaks=[s(1) s(end)];
                q_breaks=[q0 q_final_guess];
                qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_final_guess]));
                               
            end % end if (~keyframe_constraint)
            
            
            if(obj.planning_mode == 1)
            % PERFORM IKSEQUENCE OPT
            ikseq_options.Q = diag(cost(1:getNumDOF(obj.r)));
            ikseq_options.Qa = eye(getNumDOF(obj.r));
            ikseq_options.Qv = eye(getNumDOF(obj.r));
            ikseq_options.nSample = obj.num_breaks-1;
            ikseq_options.qdotf.lb = zeros(obj.r.getNumDOF(),1);
            ikseq_options.qdotf.ub = zeros(obj.r.getNumDOF(),1);
            ikseq_options.quasiStaticFlag=true;
            ikseq_options.shrinkFactor = 0.95;
            ikseq_options.jointLimitMin = ikoptions.jointLimitMin;
            ikseq_options.jointLimitMax = ikoptions.jointLimitMax;
            if(is_keyframe_constraint)
                ikseq_options.MajorIterationsLimit = 100;
                ikseq_options.qtraj0 = obj.qtraj_guess_fine; % use previous optimization output as seed
                q0 = obj.qtraj_guess_fine.eval(0); % use start of cached trajectory instead of current
            else
                    ikseq_options.MajorIterationsLimit = 100;
                    ikseq_options.qtraj0 = qtraj_guess;
              end
                ikseq_options.q_traj_nom = ikseq_options.qtraj0; % Without this the cost function is never used
                %============================
                [s_breaks,q_breaks,qdos_breaks,qddos_breaks,snopt_info] = inverseKinSequence(obj.r,q0,0*q0,ks,ikseq_options);
                if(snopt_info > 10)
                    warning('The IK sequence fails');
                    send_status(3,0,0,sprintf('snopt_info == %d. The IK sequence fails.',snopt_info));
                end
                %============================
                xtraj = PPTrajectory(pchipDeriv(s_breaks,[q_breaks;qdos_breaks],[qdos_breaks;qddos_breaks]));
                xtraj = xtraj.setOutputFrame(obj.r.getStateFrame()); %#ok<*NASGU>
            
                obj.s_breaks = s_breaks;
                obj.q_breaks = q_breaks;
                obj.qdos_breaks = qdos_breaks;
            
            qtraj_guess = PPTrajectory(spline(s_breaks,q_breaks));
            obj.qtraj_guess = qtraj_guess; % cache
            else
              if (~is_keyframe_constraint)
                obj.s_breaks = s_breaks;
                obj.q_breaks = q_breaks;
                obj.qtraj_guess = qtraj_guess; % cache
              end
            end
            
            
            % calculate end effectors breaks via FK.
            for brk =1:length(s_breaks),
                kinsol_tmp = doKinematics(obj.r,q_breaks(:,brk));
                rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_hand_body,[0;0;0],2);
                lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_hand_body,[0;0;0],2);
                rfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.r_foot_body,[0;0;0],2);
                lfoot_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.l_foot_body,[0;0;0],2);
                head_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,obj.head_body,[0;0;0],2);
            end
            
            q = q_breaks(:,1);
            
            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),s_total_head);
            s_total = max(s_total,0.01);
            
            res = 0.15; % 20cm res
            s= linspace(0,1,ceil(s_total/res)+1); % Must have two points atleast
            s = unique([s(:);s_breaks(:)]);
            
            
            
            do_second_stage_IK_verify =  false; % fine grained verification of COM constraints of fixed resolution.
            for i=2:length(s)
                si = s(i);
                %tic;
                
                if(do_second_stage_IK_verify)
                    ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
                    ikoptions.q_nom = q(:,i-1);
                    r_hand_pose_at_t=pose_spline(s_breaks,rhand_breaks,si);  %#ok<*PROP> % evaluate in quaternions
                    l_hand_pose_at_t=pose_spline(s_breaks,lhand_breaks,si); % evaluate in quaternions
                    r_foot_pose_at_t=pose_spline(s_breaks,rfoot_breaks,si);  %#ok<*PROP> % evaluate in quaternions
                    l_foot_pose_at_t=pose_spline(s_breaks,lfoot_breaks,si);
                    head_pose_at_t=pose_spline(s_breaks,head_breaks,si); % evaluate in quaternions
                    
                    
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
                    rfoot_const_static_contact = rfoot_const;
                    rfoot_const_static_contact.contact_state = ...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
                    lfoot_const_static_contact = lfoot_const;
                    lfoot_const_static_contact.contact_state = ...
                        {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
                    
                    
                    %============================
                    q_guess =qtraj_guess.eval(si);
                    %   0,comgoal,...
                    %   obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...
                    
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
                    
                    if(obj.restrict_feet)
                          %obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.r_foot_body,r_foot_pts,r_foot_pose0_static_contact,...
                            obj.l_foot_body,l_foot_pts,l_foot_pose0_static_contact,...
                            obj.r_hand_body,[0;0;0],rhand_const, ...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
%                             obj.head_body,[0;0;0],head_const,...
%                             ikoptions);
                    else
                        % obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...
                        % obj.pelvis_body,[0;0;0],pelvis_pose0,...
                        [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
                            obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,...
                            obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,...
                            obj.r_hand_body,[0;0;0],rhand_const,...
                            obj.l_hand_body,[0;0;0],lhand_const,...
                            ikoptions);
%                             obj.head_body,[0;0;0],head_const,...
%                             ikoptions);
                    end
                    %============================
                    
                    %toc;
                    if(snopt_info > 10)
                        warning(['The IK fails at ',num2str(s(i))]);
                        send_status(3,0,0,sprintf('snopt_info = %d: The IK fails at %10.4f',snopt_info,num2str(s(i))));
                    end
                    
                else
                    q(:,i) =qtraj_guess.eval(si);
                end
            end
            
            qtraj_guess_fine = PPTrajectory(spline(s, q));
            obj.qtraj_guess_fine = qtraj_guess_fine; % cache
            
            % publish robot plan
            disp('Publishing plan...');
            xtraj = zeros(getNumStates(obj.r)+2,length(s));
            xtraj(1,:) = 0*s;
            xtraj(2,:) = 0*s;
            if(length(s_breaks)>obj.num_breaks)
                keyframe_inds = unique(round(linspace(1,length(s_breaks),obj.num_breaks)));
            else
                keyframe_inds =[1:length(s_breaks)];
            end
            
            for l = keyframe_inds,
                ind = find(s == s_breaks(l));
                xtraj(1,ind) = 1.0;
            end
            xtraj(3:getNumDOF(obj.r)+2,:) = q;
            
            
            
            ts = s.*(s_total/obj.v_desired); % plan timesteps
            obj.time_2_index_scale = (obj.v_desired/s_total);
            %obj.plan_pub.publish(ts,xtraj);
            utime = now() * 24 * 60 * 60;
             % ignore the first state
             % ts = ts(2:end);
             % xtraj=xtraj(:,2:end);
            obj.plan_pub.publish(xtraj,ts,utime);
        end
        
        function generateAndPublishTeleopPlan(obj,q0,ee_delta_pos,ee_delta_rpy,ee_rhand,ee_lhand,aff2hand_offset,mate_axis)
          mate_axis_angle = acos(mate_axis'*[0;1;0]);
          mate_rotmat = quat2rotmat(axis2quat([cross(mate_axis,[0;1;0]);mate_axis_angle]));
          local_x_axis = mate_rotmat*[1;0;0];
          local_y_axis = mate_rotmat*[0;1;0];
          local_z_axis = mate_rotmat*[0;0;1];
          mate_rotmat = eye(3);
          rpalm_pts = [[0;-0.1;0] aff2hand_offset];
          lpalm_pts = [[0;0.1;0] aff2hand_offset];
          rfoot_pts = obj.r_foot_body.getContactPoints();
          lfoot_pts = obj.l_foot_body.getContactPoints();
          head_pts = [0;0;0];
          kinsol0 = doKinematics(obj.r,q0);
          rpalm_curr = forwardKin(obj.r,kinsol0,obj.r_hand_body,rpalm_pts,1);
          lpalm_curr = forwardKin(obj.r,kinsol0,obj.l_hand_body,lpalm_pts,1);
          rf_curr = forwardKin(obj.r,kinsol0,obj.r_foot_body,rfoot_pts,0);
          lf_curr = forwardKin(obj.r,kinsol0,obj.l_foot_body,lfoot_pts,0);
          head_curr = forwardKin(obj.r,kinsol0,obj.head_body,head_pts,1);
          com_curr = getCOM(obj.r,kinsol0);
          com_const = struct('max',com_curr+3e-3*ones(3,1),'min',com_curr-3e-3*ones(3,1));
          rpalm_goal = rpalm_curr(:,1);
          lpalm_goal = lpalm_curr(:,1);
          if(ee_lhand)
            lpalm_pts = lpalm_pts(:,2);
            rpalm_pts = rpalm_pts(:,1);
            lpalm_goal(1:3) = lpalm_curr(1:3,2)+ee_delta_pos;
            delta_rotmat = axis2rotmat([local_z_axis;ee_delta_rpy(3)])*axis2rotmat([local_y_axis;ee_delta_rpy(2)])*axis2rotmat([local_x_axis;ee_delta_rpy(1)]);
            lpalm_goal(4:6) = rotmat2rpy(delta_rotmat*rpy2rotmat(lpalm_curr(4:6)));
          elseif(ee_rhand)
            lpalm_pts = lpalm_pts(:,1);
            rpalm_pts = rpalm_pts(:,2);
            rpalm_goal(1:3) = rpalm_curr(1:3,2)+ee_delta_pos;
            delta_rotmat = axis2rotmat([local_z_axis;ee_delta_rpy(3)])*axis2rotmat([local_y_axis;ee_delta_rpy(2)])*axis2rotmat([local_x_axis;ee_delta_rpy(1)]);
            rpalm_goal(4:6) = rotmat2rpy(delta_rotmat*rpy2rotmat(rpalm_curr(4:6)));
          end
          ikargs = {obj.r_hand_body,rpalm_pts,rpalm_goal,obj.l_hand_body,lpalm_pts,lpalm_goal,...
            obj.r_foot_body,rfoot_pts,rf_curr,obj.l_foot_body,lfoot_pts,lf_curr,...
            obj.head_body,head_pts,head_curr,0,com_const};
          cost = diag(obj.getCostVector());
          cost = cost(1:obj.r.getNumDOF,1:obj.r.getNumDOF);
          [q_des,info] = inverseKin(obj.r,q0,ikargs{:},struct('q_nom',q0,'Q',cost));
          if(info>10)
            warning(['Info = ',num2str(info),' IK fails for teleoperation']);
            send_status(3,0,0,['Info = ',num2str(info),' IK fails for teleoperation']);
          end
          qtraj = [q0 q_des];
          xtraj = [qtraj;0*qtraj];
          xtraj = [zeros(2,2);xtraj];
          ts = [0 1];
          utime = now() * 24 * 60 * 60;
          obj.plan_pub.publish(xtraj,ts,utime);
        end
        
        function cost = getCostVector(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 100;
            cost.base_y = 100;
            cost.base_z = 100;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 100;
            cost.back_lbz = 1e4;
            cost.back_mby = 1e4;
            cost.back_ubx = 1e4;
            cost.neck_ay =  100;
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
        
        function cost = getCostVector2(obj)
            cost = Point(obj.r.getStateFrame,1);
            cost.base_x = 1;
            cost.base_y = 1;
            cost.base_z = 1;
            cost.base_roll = 100;
            cost.base_pitch = 100;
            cost.base_yaw = 1;
            cost.back_lbz = 10;
            cost.back_mby = 10000;
            cost.back_ubx = 10000;
            cost.neck_ay =  100;
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
        
    end
    
    methods (Static=true)
        
    end
    
end
