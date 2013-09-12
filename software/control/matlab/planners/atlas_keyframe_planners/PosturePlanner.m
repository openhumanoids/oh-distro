classdef PosturePlanner < KeyframePlanner
    % USAGE
    % PosturePlanner posturePlanner(r);
    % posturePlanner.generateAndPublishPlan(vargin);
    % cache = posturePlanner.getPlanCache();
    %
    properties
        plan_pub
    end
    
    methods
        function obj = PosturePlanner(r,hardware_mode)
            obj = obj@KeyframePlanner(r); % initialize the base class 
            obj.hardware_mode = hardware_mode;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
            joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
            joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
            obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);            
            obj.plan_cache.num_breaks = 4;
        end
    %-----------------------------------------------------------------------------------------------------------------              
        function generateAndPublishPosturePlan(obj,x0,q_desired,useIK_state)
            %useIK_state -  {0,1,...,5};
            %0-NoIK;
            %1-WithIK, Ensures Feet are in contact also realigns pelvis to be upright ;
            %2-WithIK, Ensures Feet are in contact, No Realignment;
            %3-Only reads Left Arm Joint Angles from the mat file (w. Pelvis Realignment);
            %4-Only reads Right Arm Joint Angles from the mat file (w. Pelvis Realignment);
            %5-For Crouching, offsets pelvis height manually so that the rfoot is on the ground(z=0);
            if(obj.isBDIManipMode())
                if (useIK_state==5)
                    warning('PosturePlanner:: you cant crouch in BDI_MANIP_MODE.');
                    send_status(4,0,0,'Warning::PosturePlanner::Cant crouch in BDI_MANIP_MODE.\n');
                end
            end
            
            runOptimizationForPosturePlan(obj,x0,q_desired,useIK_state);
        end
     %-----------------------------------------------------------------------------------------------------------------             
        function runOptimizationForPosturePlan(obj,x0,q_desired,useIK_state)
            disp('Generating posture plan...');
            q0 = x0(1:getNumDOF(obj.r));
            s = [0 1];
            nq = obj.r.getNumDOF();
            
            if(useIK_state == 1)
                kinsol_des = doKinematics(obj.r,q_desired);
                r_foot_desired = forwardKin(obj.r,kinsol_des,obj.r_foot_body,[0;0;0],0);
                base_desired_height = q_desired(3)-r_foot_desired(3);
                pelvis_des_dir = forwardKin(obj.r,kinsol_des,obj.pelvis_body,[[0;0;0] [0;0;1]],0);
                pelvis_des_dir = pelvis_des_dir(:,2)-pelvis_des_dir(:,1);
                kinsol0 = doKinematics(obj.r,q0);
                rfoot0 = forwardKin(obj.r,kinsol0,obj.r_foot_body,[0;0;0],2);
                lfoot0 = forwardKin(obj.r,kinsol0,obj.l_foot_body,[0;0;0],2);
                ikoptions = struct();
                [ikoptions.jointLimitMin,ikoptions.jointLimitMax] = obj.r.getJointLimits();
                coords = obj.r.getStateFrame.coordinates();
                coords = coords(1:obj.r.getNumDOF());
                lower_joint_ind = ~cellfun(@isempty,strfind(coords,'leg'))&cellfun(@isempty,strfind(coords,'mhx'))&cellfun(@isempty,strfind(coords,'lhy'));
                upper_joint_ind = cellfun(@isempty,strfind(coords,'leg'))&cellfun(@isempty,strfind(coords,'pelvis'))&cellfun(@isempty,strfind(coords,'base'));
                knee_joint_ind = ~cellfun(@isempty,strfind(coords,'kny'));
                ikoptions.jointLimitMin([4,5]) = q_desired([4,5]);
                ikoptions.jointLimitMin(1:2) = q0(1:2)-0.05;
                ikoptions.jointLimitMin(3) = rfoot0(3,1)+base_desired_height-0.05;
                ikoptions.jointLimitMin(upper_joint_ind) = q_desired(upper_joint_ind);
                ikoptions.jointLimitMax([4,5]) = q_desired([4,5]);
                ikoptions.jointLimitMax(1:2) = q0(1:2)+0.05;
                ikoptions.jointLimitMax(3) = rfoot0(3,1)+base_desired_height+0.0;
                ikoptions.jointLimitMax(upper_joint_ind) = q_desired(upper_joint_ind);
                ikoptions.jointLimitMin(knee_joint_ind) = 0.2;
                pelvis_const.type = 'gaze';
                pelvis_const.gaze_dir = pelvis_des_dir;
                pelvis_const.gaze_axis = [0;0;1];
                if(obj.isBDIManipMode()) % Dont adjust pelvis in BDIManipMode
                    pelvis_const = forwardKin(obj.r,kinsol0,obj.pelvis_body,[0;0;0],2);
                end
                ikargs = {obj.r_foot_body,[0;0;0],rfoot0,obj.l_foot_body,[0;0;0],lfoot0,...
                    obj.pelvis_body,[0;0;0],pelvis_const};
                cost = diag(obj.getCostVector());
                cost = cost(1:nq,1:nq);
                ikoptions.Q = cost;
                ikoptions.q_nom = q_desired;
                [q_desired,info] = inverseKin(obj.r,q_desired,ikargs{:},ikoptions);
                if(info>10)
                    send_status(3,0,0,sprintf('IK info = %d in posture plan optimization\n',info));
                end
            elseif(useIK_state ==2) % Foot in contact
                ikoptions.jointLimitMax = [inf(6,1);q_desired(7:end)];
                ikoptions.jointLimitMin = [-inf(6,1);q_desired(7:end)];
                ikoptions.shrinkFactor = 0.9;
                lfoot_const = struct();
                kinsol0 = doKinematics(obj.r,q0);
                rfoot0 = forwardKin(obj.r,kinsol0,obj.r_foot_body,[0;0;0],2);
                lfoot0 = forwardKin(obj.r,kinsol0,obj.l_foot_body,[0;0;0],2);
                ikargs = {obj.r_foot_body,[0;0;0],rfoot0,obj.l_foot_body,[0;0;0],lfoot0};
                if(obj.isBDIManipMode()) % Dont adjust pelvis in BDIManipMode
                    pelvis_const = forwardKin(obj.r,kinsol0,obj.pelvis_body,[0;0;0],2);
                    ikargs = {obj.pelvis_body,[0;0;0],pelvis_const}; % pin pelvis
                end
                ikoptions.q_nom = q0;
                [q_desired,info] = inverseKin(obj.r,q0,ikargs{:},ikoptions);
                if(info>10)
                    send_status(4,0,0,sprintf('IK info = %d in posture plan optimization\n',info));
                end
            elseif(useIK_state == 3) % copy the left arm joint angles from the mat file
                coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
                l_arm_ind = ~cellfun(@isempty,strfind(coords,'l_arm'));
                r_arm_ind = ~cellfun(@isempty,strfind(coords,'r_arm'));
                knee_joint_ind = ~cellfun(@isempty,strfind(coords,'kny'));
                [ikoptions.jointLimitMin,ikoptions.jointLimitMax] = obj.r.getJointLimits();
                ikoptions.jointLimitMin(l_arm_ind) = q_desired(l_arm_ind);
                ikoptions.jointLimitMax(l_arm_ind) = q_desired(l_arm_ind);
                ikoptions.jointLimitMin(knee_joint_ind) = 0.2;
                kinsol_des = doKinematics(obj.r,q_desired);
                rfoot_des = forwardKin(obj.r,kinsol_des,obj.r_foot_body,[0;0;0],0);
                pelvis_des_dir = forwardKin(obj.r,kinsol_des,obj.pelvis_body,[[0;0;0] [0;0;1]],0);
                pelvis_des_height = pelvis_des_dir(3,1)-rfoot_des(3,1);
                pelvis_des_dir = pelvis_des_dir(:,2)-pelvis_des_dir(:,1);
                pelvis_const.type = 'gaze';
                pelvis_const.gaze_axis = [0;0;1];
                pelvis_const.gaze_dir = pelvis_des_dir;
                kinsol_curr = doKinematics(obj.r,q0);
                rhand_curr = forwardKin(obj.r,kinsol_curr,obj.r_hand_body,[0;0;0],2);
                rhand_const.max = rhand_curr+0.03*ones(7,1);
                rhand_const.min = rhand_curr-0.03*ones(7,1);
                head_curr = forwardKin(obj.r,kinsol_curr,obj.head_body,[0;0;0],2);
                head_const.max = head_curr+0.05*ones(7,1);
                head_const.min = head_curr-0.05*ones(7,1);
                rfoot_curr = forwardKin(obj.r,kinsol_curr,obj.r_foot_body,[0;0;0],2);
                lfoot_curr = forwardKin(obj.r,kinsol_curr,obj.l_foot_body,[0;0;0],2);
                if(obj.isBDIManipMode()) % Dont adjust pelvis in BDIManipMode
                    pelvis_const = forwardKin(obj.r,kinsol_curr,obj.pelvis_body,[0;0;0],2);
                else
                    ikoptions.jointLimitMax(3) = rfoot_curr(3,1)+pelvis_des_height;
                end
                cost = diag(obj.getCostVector());
                ikoptions.Q = cost(1:obj.r.getNumDOF,1:obj.r.getNumDOF);
                ikargs = {obj.r_foot_body,[0;0;0],rfoot_curr,obj.l_foot_body,[0;0;0],lfoot_curr,...
                    obj.r_hand_body,[0;0;0],rhand_const,obj.head_body,[0;0;0],head_const,...
                    obj.pelvis_body,[0;0;0],pelvis_const};
                [q_desired,info] = inverseKin(obj.r,q0,ikargs{:},ikoptions);
                if(info>10)
                    send_status(4,0,0,sprintf('IK info = %d in posture plan optimization\n',info));
                end
            elseif(useIK_state == 4) % copy the right arm joint angles from the at file
                coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
                l_arm_ind = ~cellfun(@isempty,strfind(coords,'l_arm'));
                r_arm_ind = ~cellfun(@isempty,strfind(coords,'r_arm'));
                knee_joint_ind = ~cellfun(@isempty,strfind(coords,'kny'));
                [ikoptions.jointLimitMin,ikoptions.jointLimitMax] = obj.r.getJointLimits();
                ikoptions.jointLimitMin(r_arm_ind) = q_desired(r_arm_ind);
                ikoptions.jointLimitMax(r_arm_ind) = q_desired(r_arm_ind);
                ikoptions.jointLimitMin(knee_joint_ind) = 0.2;
                kinsol_des = doKinematics(obj.r,q_desired);
                rfoot_des = forwardKin(obj.r,kinsol_des,obj.r_foot_body,[0;0;0],0);
                pelvis_des_dir = forwardKin(obj.r,kinsol_des,obj.pelvis_body,[[0;0;0] [0;0;1]],0);
                pelvis_des_height = pelvis_des_dir(3,1)-rfoot_des(3,1);
                pelvis_des_dir = pelvis_des_dir(:,2)-pelvis_des_dir(:,1);
                pelvis_const.type = 'gaze';
                pelvis_const.gaze_axis = [0;0;1];
                pelvis_const.gaze_dir = pelvis_des_dir;
                kinsol_curr = doKinematics(obj.r,q0);
                lhand_curr = forwardKin(obj.r,kinsol_curr,obj.l_hand_body,[0;0;0],2);
                lhand_const.max = lhand_curr+0.03*ones(7,1);
                lhand_const.min = lhand_curr-0.03*ones(7,1);
                head_curr = forwardKin(obj.r,kinsol_curr,obj.head_body,[0;0;0],2);
                head_const.max = head_curr+0.05*ones(7,1);
                head_const.min = head_curr-0.05*ones(7,1);
                rfoot_curr = forwardKin(obj.r,kinsol_curr,obj.r_foot_body,[0;0;0],2);
                lfoot_curr = forwardKin(obj.r,kinsol_curr,obj.l_foot_body,[0;0;0],2);
                if(obj.isBDIManipMode()) % Dont adjust pelvis in BDIManipMode
                    pelvis_const = forwardKin(obj.r,kinsol_curr,obj.pelvis_body,[0;0;0],2);
                else
                    ikoptions.jointLimitMax(3) = rfoot_curr(3,1)+pelvis_des_height;
                end
                cost = diag(obj.getCostVector());
                ikoptions.Q = cost(1:obj.r.getNumDOF,1:obj.r.getNumDOF);
                ikargs = {obj.r_foot_body,[0;0;0],rfoot_curr,obj.l_foot_body,[0;0;0],lfoot_curr,...
                    obj.l_hand_body,[0;0;0],lhand_const,obj.head_body,[0;0;0],head_const,...
                    obj.pelvis_body,[0;0;0],pelvis_const};
                [q_desired,info] = inverseKin(obj.r,q0,ikargs{:},ikoptions);
                if(info>10)
                    send_status(4,0,0,sprintf('IK info = %d in posture plan optimization\n',info));
                end
            elseif(useIK_state == 5)
                kinsol_des = doKinematics(obj.r,q_desired);
                rfoot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
                rfoot_des = forwardKin(obj.r,kinsol_des,obj.r_foot_body,rfoot_pts,0);
                pelvis_height = q_desired(3)-rfoot_des(3,1);
                kinsol_curr = doKinematics(obj.r,q0);
                rfoot_curr = forwardKin(obj.r,kinsol_curr,obj.r_foot_body,rfoot_pts,0);%                         
                %KeyframePlanner.cacheLFootContactConstraint(obj,tspan,kc_name,lfoot_pose)
                rfoot_curr_height = min(rfoot_curr(3,:));
                pelvis_desired_height = pelvis_height+rfoot_curr_height;
                q_desired(3) = pelvis_desired_height;
                q_desired([1 2 6]) = q0([1 2 6]);
            end
            qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_desired]));
            s = linspace(0,1,4);
            s_breaks = linspace(s(1),s(end),obj.plan_cache.num_breaks);
            
            s = unique([s(:);s_breaks(:)]);
            q = zeros(length(q0),length(s));
            for i=1:length(s),
                q(:,i) = qtraj_guess.eval(s(i));
            end
            
            obj.plan_cache.s = s;    
            obj.plan_cache.s_breaks = s_breaks;
            obj.plan_cache.qtraj = PPTrajectory(spline(s, q));
            obj.plan_cache.quasiStaticFlag = false;
            
            disp('Publishing posture plan...');
            xtraj = zeros(getNumStates(obj.r)+2,length(s));
            xtraj(1,:) = 0*s;
            
            % calculate end effectors breaks via FK.
            q_breaks = zeros(nq,length(s_breaks));

            obj.plan_cache.ks = ActionSequence();
            for brk =1:length(s_breaks),
                q_breaks(:,brk) = obj.plan_cache.qtraj.eval(s_breaks(brk));
                
                if((brk==1)||brk==length(s_breaks))
                    
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
               end
            end
            
            for l =1:length(s_breaks),
                ind = find(abs(s - s_breaks(l))<1e-3);
                xtraj(1,ind) = 1.0;
                xtraj(2,ind) = 0.0;
            end
            xtraj(2+(1:nq),:) = q_breaks;
            Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
            Tmax_joints=obj.getTMaxForMaxJointSpeed();
            ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps
            obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
            utime = now() * 24 * 60 * 60;

            obj.plan_pub.publish(xtraj,ts,utime);
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
