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

        % Caches a Redundant two element plan.
        % Flag indicates KeyframeAdjustmentEngine to
        % publish an single keyframe endpose instead
        % of a keyframe plan by resolving at time T.
        obj.plan_cache.isEndPose = true;
      end
  %-----------------------------------------------------------------------------------------------------------------              
      function generateAndPublishCandidateRobotEndPose(obj,x0,ee_names,ee_loci,timeIndices,postureconstraint,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags) %#ok<INUSD>
          runPoseOptimization(obj,x0,ee_names,ee_loci,timeIndices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags);
      end
  %-----------------------------------------------------------------------------------------------------------------              
      function runPoseOptimization(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags)

        disp('Generating candidate endpose...');
        send_status(3,0,0,'Generating candidate endpose...');

        q0 = x0(1:getNumDOF(obj.r));

        T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));

        % get current hand and foot positions
        kinsol = doKinematics(obj.r,q0);

        r_foot_pts = [0;0;0];
        l_foot_pts = [0;0;0];
        r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
        l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
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


        cost = getCostVector2(obj);
        ikoptions = IKoptions(obj.r);
        ikoptions = ikoptions.setQ(diag(cost(1:getNumDOF(obj.r))));
        ikoptions = ikoptions.setDebug(true);
        ik_qnom = q0;
        ikoptions = ikoptions.setMajorIterationsLimit(1000);
        qsc = QuasiStaticConstraint(obj.r);
        qsc = qsc.setShrinkFactor(0.8);


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
        r_hand_pose = nan(7,1);
        l_hand_pose = nan(7,1);
        if(goal_type_flags.rh == 2)
          rhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.r_hand_body,[1;0;0],rh_ee_goal(1:3),[0;0;0],pi/18)};
        else
          rhand_constraint = {};
        end
        if(goal_type_flags.lh == 2)
          lhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.l_hand_body,[1;0;0],lh_ee_goal(1:3),[0;0;0],pi/18)};
        else
          lhand_constraint = {};
        end
        if(goal_type_flags.h == 2)
          head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,[1;0;0],h_ee_goal(1:3),[0;0;0],pi/12)};
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
        qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);
        %head_constraint = [head_constraint,{WorldQuatConstraint(obj.r,obj.head_body,head_pose0(4:7),1e-4)}];
%         pelvis_constraint = parse2PosQuatConstraint(obj.r,obj.pelvis_body,[0;0;0],pelvis_pose0,0,1e-2,[-inf inf]);
        pelvis_constraint = {};
%         utorso_constraint = parse2PosQuatConstraint(obj.r,obj.utorso_body,[0;0;0],utorso_pose0,0,1e-2,[-inf inf]);


        ind=find(Indices==timeIndices(1));
        for k=1:length(ind),
          if(strcmp('pelvis',ee_names{ind(k)}))
            pelvisT = ee_loci(:,ind(k));
            pelvis_constraint = {WorldQuatConstraint(obj.r,obj.pelvis_body,rpy2quat(pelvisT(4:6)),1e-2)};
          elseif(strcmp('left_palm',ee_names{ind(k)}))
            l_ee_goal = ee_loci(:,ind(k));
            lhandT = zeros(6,1);
            T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
            T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
            lhandT(1:3) = T_world_hand_l(1:3,4);
            lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
            l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
            lhand_constraint = parse2PosQuatConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose,1e-4,1e-4,[k/N,k/N]);

            obj.plan_cache.lhand_constraint_cell = [obj.plan_cache.lhand_constraint_cell lhand_constraint];

              %q_guess(1:3) = l_hand_pose(1:3);
          elseif(strcmp('right_palm',ee_names{ind(k)}))
            r_ee_goal = ee_loci(:,ind(k));
            rhandT = zeros(6,1);
            T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
            T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
            rhandT(1:3) = T_world_hand_r(1:3,4);
            rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
            r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
            rhand_constraint = parse2PosQuatConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose,1e-4,1e-4,[k/N,k/N]);

            obj.plan_cache.rhand_constraint_cell = [obj.plan_cache.rhand_constraint_cell rhand_constraint];

          elseif (strcmp('l_foot',ee_names{ind(k)}))
            lfootT = ee_loci(:,ind(k));
            %l_foot_pose = [lfootT(1:3); rpy2quat(lfootT(4:6))];
            %lfoot_const.min = l_foot_pose-1e-6*[ones(3,1);ones(4,1)];
            %lfoot_const.max = l_foot_pose+1e-6*[ones(3,1);ones(4,1)];
            l_foot_pose = lfootT(1:3);
            % Ask Sisir why there is no orientation constraint
            lfoot_constraint = parse2PosQuat(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose,1e-6,1e-6,[-inf,inf]);
          elseif(strcmp('r_foot',ee_names{ind(k)}))
            rfootT = ee_loci(:,ind(k));
            %r_foot_pose = [rfootT(1:3); rpy2quat(rfootT(4:6))];
            %rfoot_const.min = r_foot_pose-1e-6*[ones(3,1);ones(4,1)];
            %rfoot_const.max = r_foot_pose+1e-6*[ones(3,1);ones(4,1)];
            r_foot_pose = rfootT(1:3);
            rfoot_constraint = parse2PosQuat(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose,1e-6,1e-6,[-inf,inf]);
          else
            disp('currently only feet/hands and pelvis are allowed');
          end
        end
        qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);


     


        ikoptions = ikoptions.setQ( diag(cost(1:getNumDOF(obj.r))));

        joint_constraint = PostureConstraint(obj.r);
        %nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_comfortable_right_arm_manip.mat'));
        nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
        qstar = nomdata.xstar(1:obj.r.getNumDOF());
        ik_qnom = qstar;
        %  			ikoptions.q_nom = qstar;
        NSamples = 10;
        yaw_angles_bnd = 25;
        arm_loci_flag = ~cellfun(@(x) isempty(strfind(char(x),'palm')),ee_names);
        if(sum(arm_loci_flag) == 2)
          nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_two_hands_reaching.mat'));
          qstar = nomdata.xstar(1:obj.r.getNumDOF());
          ik_qnom = qstar;
          rhand_constraint = {WorldPositionConstraint(obj.r,obj.r_hand_body,[0;0;0],r_hand_pose(1:3)-0.03,r_hand_pose(1:3)+0.03)};
          if(all(~isnan(r_hand_pose(4:7))))
            rhand_constraint = [rhand_constraint,{WorldQuatConstraint(obj.r,obj.r_hand_body,r_hand_pose(4:7),1e-1)}];
          end
          lhand_const = {WorldPositionConstraint(obj.r,obj.l_hand_body,[0;0;0],l_hand_pose(1:3)-0.03,l_hand_pose(1:3)+0.03)};
          if(all(~isnan(l_hand_pose(4:7))))
            lhand_const = [lhand_const,{WorldQuatConstraint(obj.r,obj.l_hand_body,l_hand_pose(4:7),1e-1)}];
          end
          qsc = qsc.setShrinkFactor(0.95);
          cost = diag(obj.getCostVector3());
          cost = cost(1:obj.r.getNumDOF(),1:obj.r.getNumDOF());
          ikoptions = ikoptions.setQ(cost);
          coords = obj.r.getStateFrame.coordinates;
          coords = coords(1:obj.r.getNumDOF);
          back_bky_ind =  find(strcmp(coords,'back_bky'));
          r_arm_usy_ind = find(strcmp(coords,'r_arm_usy'));
          l_arm_usy_ind = find(strcmp(coords,'l_arm_usy'));
          r_leg_kny_ind = find(strcmp(coords,'r_leg_kny'));
          l_leg_kny_ind = find(strcmp(coords,'l_leg_kny'));
          joint_constraint = joint_constraint.setJointLimits([back_bky_ind;r_arm_usy_ind;l_arm_usy_ind;r_leg_kny_ind;l_leg_kny_ind],...
            [-0.3;-inf;-inf;0.2;0.2],[0.3;0;0;inf;inf]);
          [joint_min,joint_max] = joint_constraint.bounds([]);
          coords = obj.r.getStateFrame.coordinates();
          coords = coords(1:obj.r.getNumDOF());
          arm_joint_ind = find(~cellfun(@isempty,strfind(coords,'arm')));
          joint_constraint = joint_constraint.setJointLimits(arm_joint_ind,0.9*joint_min(arm_joint_ind),0.9*joint_max(arm_joint_ind));
          NSamples = 20;
          yaw_samples_bnd = 60;
        end
        q_sample = zeros(obj.r.getNumDOF,NSamples);
        sample_cost = zeros(1,NSamples);
        for k=1:NSamples,
          %q_guess = qstar;
          q_guess(3) = q_guess(3)+2*(rand(1,1)-0.5)*(0.2);
          q_guess(6)=q_guess(6)+2*(rand(1,1)-0.5)*(yaw_angles_bnd*pi/180);%+-10degrees from current pose
          if(~obj.isBDIManipMode())
            qsc = qsc.setActive(true);
          else
            qsc = qsc.setActive(false);
          end
          cost = diag(obj.getCostVector2());
          cost = cost(1:obj.r.getNumDOF(),1:obj.r.getNumDOF());
          ikoptions = ikoptions.setQ(cost);
          coords = obj.r.getStateFrame.coordinates;
          coords = coords(1:obj.r.getNumDOF);
          joint_ind = (1:obj.r.getNumDOF)';
          back_bky_ind = joint_ind(strcmp(coords,'back_bky'));
          r_leg_kny_ind = joint_ind(strcmp(coords,'r_leg_kny'));
          l_leg_kny_ind = joint_ind(strcmp(coords,'l_leg_kny'));
          joint_constraint = joint_constraint.setJointLimits([back_bky_ind;r_leg_kny_ind;l_leg_kny_ind],...
            [-0.3;0.2;0.2],[0.3;inf;inf]);
          %obj.pelvis_body,[0;0;0],pelvis_const,...
          %   obj.head_body,[0;0;0],head_pose0_relaxed,...
          %   obj.utorso_body,[0;0;0],utorso_pose0_relaxed,...

          if(obj.isBDIManipMode()) % replace feet with pelvis in BDI Manip Mode                   

            if(~isempty(head_constraint))
              [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                pelvis_constraint{:},rhand_constraint{:},lhand_constraint{:},head_constraint{:},...
                joint_constraint,qsc,ikoptions);
            else
              [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                pelvis_constraint{:},rhand_constraint{:},lhand_constraint{:},...
                joint_constraint,qsc,ikoptions);
            end                   
          else
            if(~isempty(head_constraint))
              [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                rhand_constraint{:},lhand_constraint{:},rfoot_constraint{:},lfoot_constraint{:},head_constraint{:},...
                joint_constraint,qsc,ikoptions);
            else
              [q_sample(:,k),snopt_info,infeasible_constraint] = inverseKin(obj.r,q_guess,ik_qnom,...
                rhand_constraint{:},lhand_constraint{:},rfoot_constraint{:},lfoot_constraint{:},...
                joint_constraint,qsc,ikoptions);
            end
          end

          if(snopt_info > 10)
              warning(['poseOpt IK fails']);
              send_msg = sprintf('snopt_info = %d...\n,%s',snopt_info,infeasibleConstraintMsg(infeasible_constraint));
              send_status(4,0,0,send_msg);
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
        xtraj = zeros(getNumStates(obj.r),1);
        xtraj(1:getNumDOF(obj.r),:) = q_out;
        obj.pose_pub.publish(xtraj,utime);

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
        else
          % TODO: VERIFY
          obj.plan_cache.rfoot_constraint_cell = [obj.plan_cache.rfoot_constraint_cell rfoot_constraint];
          obj.plan_cache.lfoot_constraint_cell = [obj.plan_cache.lfoot_constraint_cell lfoot_constraint];
          obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);
        end
        

      end  % end function
  %-----------------------------------------------------------------------------------------------------------------              
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
  %-----------------------------------------------------------------------------------------------------------------              
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
  %-----------------------------------------------------------------------------------------------------------------              
  end% end methods
end% end classdef
