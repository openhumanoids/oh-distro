classdef KeyframePlanner < handle
    % Base class for Keyframe Planners that work with
    % KeyframeAdjustmentEngine class
    % USAGE:
    % SOMEPlanner xPlanner(r);
    % cache = xPlanner.getPlanCache();
    % adjustmentEngine.setPlanCache(cache);
    % adjustmentEngine.adjustAndPublishPlan(keyframe_constraint);
    properties
        r
        atlas
        plan_cache;
        hardware_mode % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User

        
        r_hand_body
        l_hand_body
        head_body
        l_foot_body
        r_foot_body
        pelvis_body
        utorso_body
        l_uarm_body 
        r_uarm_body 
        l_lleg_body
        r_lleg_body
        
        joint_constraint
        joint_constraint_args
        plan_arc_res
        
        l_hand_mode
        r_hand_mode        
        T_palm_hand_l
        T_palm_hand_r
        T_hand_palm_l
        T_hand_palm_r
        T_hand_palm_l_sandia
        T_hand_palm_r_sandia
        T_hand_palm_l_irobot
        T_hand_palm_r_irobot
        T_hand_palm_l_robotiq
        T_hand_palm_r_robotiq
        T_hand_palm_r_hose_irobot
        sandia_gaze_axis
        irobot_gaze_axis
        robotiq_gaze_axis
        lh_name
        rh_name
        lh_gaze_axis
        rh_gaze_axis
        hand_gaze_tol
        head_gaze_tol
        lidar_gaze_tol
        
        head_gaze_axis
        collision_check
        atlas_frame
        lhand_frame
        rhand_frame
        atlas2robotFrameIndMap % atlas2robotFrameMap(i) is the index of atlas.frame.coordinate{i} in the robot frame
        lhand2robotFrameIndMap
        rhand2robotFrameIndMap
        
        h_camera_origin;
        lh_camera_origin;
        rh_camera_origin;
        r_sandia_camera_origin;
        l_sandia_camera_origin;
        r_irobot_camera_origin;
        l_irobot_camera_origin;
        r_robotiq_camera_origin;
        l_robotiq_camera_origin;
        l_arm_joint_ind;
        r_arm_joint_ind;
        lower_joint_ind; %leg joints and floating base;
        
        planner_config_publisher;
    end
    
    methods
        function obj = KeyframePlanner(r,atlas,lhand_frame,rhand_frame)
            obj.r = r;
            obj.plan_cache = KeyframePlanCache(r);
            obj.hardware_mode = 1;
            obj.r_hand_body = findLinkInd(obj.r,'r_hand');
            obj.l_hand_body = findLinkInd(obj.r,'l_hand');
            obj.r_foot_body = obj.r.findLinkInd('r_foot');
            obj.l_foot_body = obj.r.findLinkInd('l_foot');
            obj.head_body = obj.r.findLinkInd('head');
            obj.pelvis_body = findLinkInd(obj.r,'pelvis');
            obj.utorso_body = findLinkInd(obj.r,'utorso');
            obj.l_uarm_body = findLinkInd(obj.r,'l_uarm');
            obj.r_uarm_body = findLinkInd(obj.r,'r_uarm');
            obj.l_lleg_body= findLinkInd(obj.r,'l_lleg');
            obj.r_lleg_body= findLinkInd(obj.r,'r_lleg');

           
           
            obj.setDefaultJointConstraint();
            % used to determine the length of posture and reaching plans as 
            % a function of arclength.                           
            obj.plan_arc_res = 0.05; % 5cm res                         

            % Goals are presented in palm frame, must be transformed to hand coordinate frame
            % Using notation similar to KDL.
            % fixed transform between hand and palm as specified in the urdf
            %ft_sensor_offset = 0.03516; 
      
            obj.T_hand_palm_l_sandia = HT([0.00179;0.13516;0.01176],0,0,1.57079);
            obj.T_hand_palm_r_sandia = HT([-0.00179;-0.13516;-0.01176],0,0,-1.57079);

            obj.T_hand_palm_l_irobot = HT([0;0.11516;0.015],1.57079,3.14159,3.14159);
            obj.T_hand_palm_r_irobot = HT([0;-0.11516;-0.015],1.57079,0,0);
 
% might be useful for the extenders           
%            obj.T_hand_palm_l_irobot = HT([0;0.322;0.015],1.57079,3.14159,3.14159);
%            obj.T_hand_palm_r_irobot = HT([0;-0.322;-0.015],1.57079,0,0);


            obj.T_hand_palm_r_hose_irobot = HT([0;-0.11516;-0.015],1.57079,1.57079,0);
            
            % these need to be verified on hardware after mounting brackets are designed,
            obj.T_hand_palm_l_robotiq = HT([0;0.11516;0.015],0,0,0);
            obj.T_hand_palm_r_robotiq = HT([0;-0.11516;-0.015],0,3.14159,3.14159);
            
            obj.sandia_gaze_axis = [0;0;1];
            obj.irobot_gaze_axis = [0;1;0];
            obj.robotiq_gaze_axis = [0;1;0];
            obj.head_gaze_axis = [1;0;0];
            obj.l_sandia_camera_origin = [0;0.2;0];
            obj.r_sandia_camera_origin = [0;-0.2;0];
            obj.l_irobot_camera_origin = [0;0;0];
            obj.r_irobot_camera_origin = [0;0;0];
            obj.l_robotiq_camera_origin = [0;0;0];
            obj.r_robotiq_camera_origin = [0;0;0];  
            obj.hand_gaze_tol = pi/18;
            obj.head_gaze_tol = pi/16;% FOV for multisense is 80x45, so 45/2 should be the max tolerance
            obj.lidar_gaze_tol = pi/2.5;
            obj.h_camera_origin = [0;0;0];

            obj.setHandType(lhand_frame,rhand_frame);
            % obj.collision_check            
            % - 0, no validation, no optimizatoin with collision
            % - 1, validation only, no optimization
            % - 2, optimize without collision constraint first, then validate. 
            obj.collision_check = 0;
            
            obj.atlas = atlas;
            obj.atlas2robotFrameIndMap = zeros(atlas.getNumStates,1);
            obj.atlas_frame = atlas.getStateFrame;
            obj.lhand_frame = lhand_frame;
            obj.rhand_frame = rhand_frame;
            obj.lhand2robotFrameIndMap = zeros(length(obj.lhand_frame.coordinates),1);
            obj.rhand2robotFrameIndMap = zeros(length(obj.rhand_frame.coordinates),1);
            for i = 1:atlas.getNumStates
              obj.atlas2robotFrameIndMap(i) = find(strcmp(atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
            end
            for i = 1:length(lhand_frame.coordinates)
              obj.lhand2robotFrameIndMap(i) = find(strcmp(lhand_frame.coordinates{i},obj.r.getStateFrame.coordinates));
            end
            for i = 1:length(rhand_frame.coordinates)
              obj.rhand2robotFrameIndMap(i) = find(strcmp(rhand_frame.coordinates{i},obj.r.getStateFrame.coordinates));
            end
            joint_ind = (1:obj.r.getNumDOF)';
            coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
            obj.l_arm_joint_ind = joint_ind(cellfun(@(s) ~isempty(strfind(s,'l_arm')),coords));
            obj.r_arm_joint_ind = joint_ind(cellfun(@(s) ~isempty(strfind(s,'r_arm')),coords));
            obj.lower_joint_ind = joint_ind(cellfun(@(s) ~isempty(strfind(s,'leg')) | ~isempty(strfind(s,'base')),coords));
            
            obj.planner_config_publisher = ManipPlannerConfigPublisher('PLANNER_CONFIG');
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function [cache] = getPlanCache(obj)
            cache  = obj.plan_cache;
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function setVDesired(obj,val)
            obj.plan_cache.v_desired  = val;
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function lh_name=get_lhname(obj)
            lh_name=obj.lh_name;
        end     
        function rh_name=get_rhname(obj)
            rh_name=obj.rh_name;
        end       
     %-----------------------------------------------------------------------------------------------------------------
        function setQdotDesired(obj,val)
            obj.plan_cache.qdot_desired  = val;
        end       
     %-----------------------------------------------------------------------------------------------------------------        
        function setHandType(obj,lhand_frame,rhand_frame)
          if(~isempty(strfind(lhand_frame.name,'no')))
            obj.l_hand_mode = 0;
            display('No left hand');
          elseif(~isempty(strfind(lhand_frame.name,'sandia')))
            obj.l_hand_mode = 1;
            display('Sandia left hand');
          elseif(~isempty(strfind(lhand_frame.name,'irobot')))
            obj.l_hand_mode = 2;
            display('iRobot left hand');
          elseif(~isempty(strfind(lhand_frame.name,'robotiq')))
            obj.l_hand_mode = 4;
            display('Robotiq left hand');            
          end
          if(~isempty(strfind(rhand_frame.name,'no')))
            obj.r_hand_mode = 0;
            display('No right hand');
          elseif(~isempty(strfind(rhand_frame.name,'sandia')))
            obj.r_hand_mode = 1;
            display('Sandia right hand');
          elseif(~isempty(strfind(rhand_frame.name,'irobot')))
            if(~isempty(strfind(rhand_frame.name,'Hose')))
              obj.r_hand_mode = 3;
              display('irobot right hand for hose');
            else
              obj.r_hand_mode = 2;
              display('iRobot right hand');
            end
          elseif(~isempty(strfind(rhand_frame.name,'robotiq')))
            obj.r_hand_mode = 4;
            display('Robotiq right hand');     
          end
            
            if(obj.l_hand_mode == 1)
             obj.T_hand_palm_l = obj.T_hand_palm_l_sandia;
             obj.lh_gaze_axis = obj.sandia_gaze_axis;
             obj.lh_camera_origin = obj.l_sandia_camera_origin;
            elseif(obj.l_hand_mode == 2)
             obj.T_hand_palm_l = obj.T_hand_palm_l_irobot;
              obj.lh_gaze_axis = obj.irobot_gaze_axis;
              obj.lh_camera_origin = obj.l_irobot_camera_origin;
            elseif(obj.l_hand_mode == 0)
              % I need to talk with Sisir about this palm-hand
              % transformation
              obj.T_hand_palm_l = obj.T_hand_palm_l_irobot; 
               obj.lh_gaze_axis = obj.irobot_gaze_axis;
               obj.lh_camera_origin = obj.l_irobot_camera_origin;
            elseif(obj.l_hand_mode == 4)
              % I need to talk with Sisir about this palm-hand
              % transformation
              obj.T_hand_palm_l = obj.T_hand_palm_l_robotiq; 
               obj.lh_gaze_axis = obj.robotiq_gaze_axis;
               obj.lh_camera_origin = obj.l_robotiq_camera_origin;
            end
            
            if(obj.r_hand_mode == 1)
              obj.T_hand_palm_r = obj.T_hand_palm_r_sandia;
              obj.rh_gaze_axis = obj.sandia_gaze_axis;
              obj.rh_camera_origin = obj.r_sandia_camera_origin;
            elseif(obj.r_hand_mode == 2)
              obj.T_hand_palm_r = obj.T_hand_palm_r_irobot;
              obj.rh_gaze_axis = obj.irobot_gaze_axis;
              obj.rh_camera_origin = obj.r_irobot_camera_origin;
            elseif(obj.r_hand_mode == 0)
              obj.T_hand_palm_r = obj.T_hand_palm_r_irobot;
              obj.rh_gaze_axis = obj.irobot_gaze_axis;
              obj.rh_camera_origin = obj.r_irobot_camera_origin;
            elseif(obj.r_hand_mode == 3)
              obj.T_hand_palm_r = obj.T_hand_palm_r_hose_irobot;
              obj.rh_gaze_axis = obj.irobot_gaze_axis;
              obj.rh_camera_origin = obj.r_irobot_camera_origin;
            elseif(obj.r_hand_mode == 4)
              % I need to talk with Sisir about this palm-hand
              % transformation
              obj.T_hand_palm_r = obj.T_hand_palm_r_robotiq; 
               obj.rh_gaze_axis = obj.robotiq_gaze_axis;
               obj.rh_camera_origin = obj.r_robotiq_camera_origin;
            end
            
                        
            obj.lh_name='';
            if(obj.l_hand_mode==0 || obj.l_hand_mode==2 || obj.l_hand_mode == 3)
                obj.lh_name='left_base_link';
            elseif(obj.l_hand_mode==1 || obj.l_hand_mode==4 )
                obj.lh_name='left_palm';
            end
            obj.rh_name='';
            if(obj.r_hand_mode==0 || obj.r_hand_mode==2 || obj.r_hand_mode == 3)
                obj.rh_name='right_base_link';
            elseif(obj.r_hand_mode==1 || obj.r_hand_mode==4 )
                obj.rh_name='right_palm';
            end
            
            
            obj.T_palm_hand_l = inv_HT(obj.T_hand_palm_l);
            obj.T_palm_hand_r = inv_HT(obj.T_hand_palm_r);
        end                  
     %-----------------------------------------------------------------------------------------------------------------        
        function setHardwareMode(obj,mode)
            obj.hardware_mode = mode;
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function val=isSimMode(obj)
            val = (obj.hardware_mode == 1);
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function val=isBDIManipMode(obj)
            val = (obj.hardware_mode == 2);
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function val=isBDIUserMode(obj)
            val = (obj.hardware_mode == 3);
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function Tmax_ee=getTMaxForMaxEEArcSpeed(obj,s_breaks,q_breaks)
            rhand_breaks = zeros(7,length(s_breaks));
            lhand_breaks = zeros(7,length(s_breaks));
            head_breaks = zeros(7,length(s_breaks));
            rfoot_breaks = zeros(7,length(s_breaks));
            lfoot_breaks = zeros(7,length(s_breaks));
            ruarm_breaks = zeros(7,length(s_breaks));
            luarm_breaks = zeros(7,length(s_breaks));
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

            s_total_lh =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
            s_total_rh =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
            s_total_lf =  sum(sqrt(sum(diff(lfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_rf =  sum(sqrt(sum(diff(rfoot_breaks(1:3,:),1,2).^2,1)));
            s_total_lel =  sum(sqrt(sum(diff(luarm_breaks(1:3,:),1,2).^2,1)));
            s_total_rel =  sum(sqrt(sum(diff(ruarm_breaks(1:3,:),1,2).^2,1)));
            s_total_head =  sum(sqrt(sum(diff(head_breaks(1:3,:),1,2).^2,1)));
            s_total = max(max(max(s_total_lh,s_total_rh),max(s_total_lf,s_total_rf)),max(s_total_head,max(s_total_lel,s_total_rel)));
            s_total = max(s_total,0.01);
            Tmax_ee  = (s_total/obj.plan_cache.v_desired);
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function Tmax_joints=getTMaxForMaxJointSpeed(obj)
            dqtraj=fnder(obj.plan_cache.qtraj,1); 
            sfine = linspace(0,1,50);
            coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
            neck_idx = strcmp(coords,'neck_ay');
            qdot_breaks = dqtraj.eval(sfine);
            Tmax_joints = max(max(abs(qdot_breaks(~neck_idx,:)),[],2))/obj.plan_cache.qdot_desired;
         end
     %-----------------------------------------------------------------------------------------------------------------        
        function cachePelvisPose(obj,tspan,pose)
            kc_pelvis = wrapDeprecatedConstraint(obj.r,obj.pelvis_body,[0;0;0],pose,struct('tspan',tspan,'use_mex',false));
            obj.plan_cache.pelvis_constraint_cell = [obj.plan_cache.pelvis_constraint_cell kc_pelvis];
        end
       
     %-----------------------------------------------------------------------------------------------------------------        
        function cacheLHandPose(obj,tspan,pose)
            kc_lhand = wrapDeprecatedConstraint(obj.r,obj.l_hand_body,[0;0;0],pose,struct('tspan',tspan,'use_mex',false));
            obj.plan_cache.lhand_constraint_cell = [obj.plan_cache.lhand_constraint_cell kc_lhand];
        end
      %-----------------------------------------------------------------------------------------------------------------       
        function cacheRHandPose(obj,tspan,pose)
            kc_rhand = wrapDeprecatedConstraint(obj.r,obj.r_hand_body,[0;0;0],pose,struct('tspan',tspan,'use_mex',false));
            obj.plan_cache.rhand_constraint_cell = [obj.plan_cache.rhand_constraint_cell kc_rhand];
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function cacheLFootPose(obj,tspan,l_foot_pose)
            kc_lfoot = wrapDeprecatedConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose,struct('tspan',tspan,'use_mex',false));
            obj.plan_cache.lfoot_constraint_cell = [obj.plan_cache.lfoot_constraint_cell kc_lfoot];
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function cacheRFootPose(obj,tspan,r_foot_pose)
            kc_rfoot = wrapDeprecatedConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose,struct('tspan',tspan,'use_mex',false));
            obj.plan_cache.rfoot_constraint_cell = [obj.plan_cache.rfoot_constraint_cell kc_rfoot];
        end
      %-----------------------------------------------------------------------------------------------------------------       
        function cacheLFootPoseAsContactConstraint(obj,tspan,l_foot_pose)
            l_foot_contact_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            l_foot_pts = [0;0;0]; 
            num_l_foot_pts = size(l_foot_pts,2);
            lfoot_const1 = WorldPositionConstraint(obj.r,obj.l_foot_body,l_foot_pts,...
              l_foot_pose(1:3)-1e-6*ones(3,num_l_foot_pts),l_foot_pose(1:3)+1e-6*ones(3,num_l_foot_pts),tspan);
            lfoot_const2 = WorldQuatConstraint(obj.r,obj.l_foot_body,l_foot_pose(4:7,1),1e-6,tspan);
            obj.plan_cache.lfoot_constraint_cell = [obj.plan_cache.lfoot_constraint_cell,{lfoot_const1,lfoot_const2}];
            obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.l_foot_body,l_foot_contact_pts);
        end
     %-----------------------------------------------------------------------------------------------------------------        
        function cacheRFootPoseAsContactConstraint(obj,tspan,r_foot_pose)
            r_foot_contact_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            r_foot_pts = [0;0;0]; 
            num_r_foot_pts = size(r_foot_pts,2);
            rfoot_const1 = WorldPositionConstraint(obj.r,obj.r_foot_body,r_foot_pts,...
              r_foot_pose(1:3)-1e-6*ones(3,num_r_foot_pts),r_foot_pose(1:3)+1e-6*ones(3,num_r_foot_pts),tspan);
            rfoot_const2 = WorldQuatConstraint(obj.r,obj.r_foot_body,r_foot_pose(4:7,1),1e-6,tspan);
            obj.plan_cache.rfoot_constraint_cell = [obj.plan_cache.rfoot_constraint_cell,{rfoot_const1,rfoot_const2}];
            obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,r_foot_contact_pts);
        end
     %-----------------------------------------------------------------------------------------------------------------        
       
        
        function replaceCachedConstraint(obj,body_ind,tspan,new_pose)
          new_pose_constraint = wrapDeprecatedConstraint(obj.r,body_ind,[0;0;0],new_pose,struct('use_mex',false,'tspan',tspan));
          if(body_ind == obj.l_hand_body)
            obj.plan_cache.lhand_constraint_cell = replaceConstraintCell(obj.plan_cache.lhand_constraint_cell,new_pose_constraint);
          elseif(body_ind == obj.r_hand_body)
            obj.plan_cache.rhand_constraint_cell = replaceConstraintCell(obj.plan_cache.rhand_constraint_cell,new_pose_constraint);
          elseif(body_ind == obj.head_body)
            obj.plan_cache.head_constraint_cell = replaceConstraintCell(obj.plan_cache.head_constraint_cell,new_pose_constraint);
          elseif(body_ind == obj.pelvis_body)
            obj.plan_cache.pelvis_constraint_cell = replaceConstraintCell(obj.plan_cache.pelvis_constraint_cell,new_pose_constraint);
          elseif(body_ind == obj.l_foot_body)
            obj.plan_cache.lfoot_constraint_cell = replaceConstraintCell(obj.plan_cache.lfoot_constraint_cell,new_pose_constraint);
            lfoot_contact_pts = obj.r.getBody(obj.l_foot_body).getContactPoints();
            obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.l_foot_body,lfoot_contact_pts);
          elseif(body_ind == obj.r_foot_body)
            obj.plan_cache.rfoot_constraint_cell = replaceConstraintCell(obj.plan_cache.rfoot_constraint_cell,new_pose_constraint);
            rfoot_contact_pts = obj.r.getBody(obj.r_foot_body).getContactPoints();
            obj.plan_cache.qsc = obj.plan_cache.qsc.addContact(obj.r_foot_body,rfoot_contact_pts);
          end
            
        end
     %-----------------------------------------------------------------------------------------------------------------        
        
        
        function removeBodyConstraints(obj,body_ind,tspan)
          
          if(body_ind == obj.l_hand_body)
            obj.plan_cache.lhand_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.lhand_constraint_cell);
          elseif(body_ind == obj.r_hand_body)
            obj.plan_cache.rhand_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.rhand_constraint_cell);
          elseif(body_ind == obj.l_foot_body)
            obj.plan_cache.lfoot_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.lfoot_constraint_cell);
          elseif(body_ind == obj.r_foot_body)
            obj.plan_cache.rfoot_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.rfoot_constraint_cell);
          elseif(body_ind == obj.pelvis_body)
            obj.plan_cache.pelvis_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.pelvis_constraint_cell);
          elseif(body_ind == obj.head_body)
            obj.plan_cache.head_constraint_cell = removeBodyConstraintUtil(tspan,obj.plan_cache.head_constraint_cell);
          end
        end
        
        function updateRobot(obj,r)
          obj.r = r;
          obj.plan_cache.updateRobot(r);
          obj.hardware_mode = 1;
          obj.r_hand_body = findLinkInd(obj.r,'r_hand');
          obj.l_hand_body = findLinkInd(obj.r,'l_hand');
          obj.r_foot_body = obj.r.findLinkInd('r_foot');
          obj.l_foot_body = obj.r.findLinkInd('l_foot');
          obj.head_body = obj.r.findLinkInd('head');
          obj.pelvis_body = findLinkInd(obj.r,'pelvis');
          obj.utorso_body = findLinkInd(obj.r,'utorso');
          obj.l_uarm_body = findLinkInd(obj.r,'l_uarm');
          obj.r_uarm_body = findLinkInd(obj.r,'r_uarm');


          obj.joint_constraint = PostureConstraint(obj.r);
          [joint_min,joint_max] = obj.joint_constraint.bounds([]);
          coords = obj.r.getStateFrame.coordinates;
          back_bky_ind = find(strcmp(coords,'back_bky'));
          back_bkx_ind = find(strcmp(coords,'back_bkx'));
          l_leg_kny_ind = find(strcmp(coords,'l_leg_kny'));
          r_leg_kny_ind = find(strcmp(coords,'r_leg_kny'));
          buffer=0.05;
          obj.joint_constraint = obj.joint_constraint.setJointLimits(...
            [back_bky_ind;back_bkx_ind;l_leg_kny_ind;r_leg_kny_ind],...
            [-0.1;-0.1;0.2;0.2],...
            [0.1;0.1;joint_max(l_leg_kny_ind)-buffer;joint_max(r_leg_kny_ind)-buffer]);

          obj.joint_constraint_args ={[back_bky_ind;back_bkx_ind;l_leg_kny_ind;r_leg_kny_ind],...
                                        [-0.1;-0.1;0.2;0.2],...
                                        [0.1;0.1;joint_max(l_leg_kny_ind)-buffer;joint_max(r_leg_kny_ind)-buffer]};
          for i = 1:length(obj.atlas_frame.coordinates)
            obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas_frame.coordinates{i},obj.r.getStateFrame.coordinates));
          end
          for i = 1:length(obj.lhand_frame.coordinates)
            obj.lhand2robotFrameIndMap(i) = find(strcmp(obj.lhand_frame.coordinates{i},obj.r.getStateFrame.coordinates));
          end
          for i = 1:length(obj.lhand_frame.coordinates)
            obj.rhand2robotFrameIndMap(i) = find(strcmp(obj.rhand_frame.coordinates{i},obj.r.getStateFrame.coordinates));
          end
        end
        
        function q_bound = checkPosture(obj,q)
          % Check if q is outside of the robot default joint limits
          [lb,ub] = obj.r.getJointLimits();
          coords = obj.r.getStateFrame.coordinates;
          coords = coords(1:obj.r.getNumDOF);
          joint_idx = (1:obj.r.getNumDOF)';
          lb_err = lb-q;
          ub_err = q-ub;
          lb_err_idx = joint_idx(lb_err>0);
          ub_err_idx = joint_idx(ub_err>0);
          if(~isempty(lb_err_idx))
            for i = 1:length(lb_err_idx)
              warning('Joint %s is below lower bound by %5.3f',coords{lb_err_idx(i)},lb_err(lb_err_idx(i)));
            end
          end
          if(~isempty(ub_err_idx))
            for i = 1:length(ub_err_idx)
              warning('Joint %s is above upper bound by %5.3f',coords{ub_err_idx(i)},ub_err(ub_err_idx(i)));
            end
          end
          q_bound = max([q lb],[],2);
          q_bound = min([q_bound ub],[],2);
        end
        
        function setDefaultJointConstraint(obj)
          obj.joint_constraint = PostureConstraint(obj.r);
          [joint_min,joint_max] = obj.joint_constraint.bounds([]);
          coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
          back_bky_ind = find(strcmp(coords,'back_bky'));
          back_bkx_ind = find(strcmp(coords,'back_bkx'));
          l_leg_kny_ind = find(strcmp(coords,'l_leg_kny'));
          r_leg_kny_ind = find(strcmp(coords,'r_leg_kny'));
          buffer=0.05;
          obj.joint_constraint = obj.joint_constraint.setJointLimits(...
            [back_bky_ind;back_bkx_ind;l_leg_kny_ind;r_leg_kny_ind],...
            [-0.1;-0.1;0.2;0.2],...
            [0.1;0.1;joint_max(l_leg_kny_ind)-buffer;joint_max(r_leg_kny_ind)-buffer]);

          obj.joint_constraint_args ={[back_bky_ind;back_bkx_ind;l_leg_kny_ind;r_leg_kny_ind],...
                                        [-0.1;-0.1;0.2;0.2],...
                                        [0.1;0.1;joint_max(l_leg_kny_ind)-buffer;joint_max(r_leg_kny_ind)-buffer]};
        end
        
        function BDI_joint_constraint = setBDIJointLimits(obj,joint_constraint,q0)
          % Fix the floating base and lower body joints to q0
          coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
          joint_idx = (1:obj.r.getNumDOF)';
          lower_joint_idx = joint_idx(cellfun(@(s) ~isempty(strfind(s,'leg')),coords));
          fixed_joint_idx = [(1:6)';lower_joint_idx];
          BDI_joint_constraint = joint_constraint;
          BDI_joint_constraint = BDI_joint_constraint.setJointLimits(fixed_joint_idx,q0(fixed_joint_idx),q0(fixed_joint_idx));
        end
        
        function data = checkPlannerConfig(obj,plan_execution_time)
          data = struct();
          data.desired_ee_arc_speed = obj.plan_cache.v_desired;
          data.desired_joint_speed = obj.plan_cache.qdot_desired;
          data.plan_execution_time = plan_execution_time;
        end
    end
     %-----------------------------------------------------------------------------------------------------------------
end
