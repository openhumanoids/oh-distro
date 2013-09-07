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
        
        joint_min
        joint_max
    end
    
    methods
        function obj = KeyframePlanner(r)
            obj.r = r;
            obj.plan_cache = KeyframePlanCache();
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

            coords = obj.r.getStateFrame();
            [joint_min,joint_max] = obj.r.getJointLimits();
            joint_min = Point(coords,[joint_min;0*joint_min]);
            joint_min.back_bky = -0.1;
            joint_min.back_bkx = -0.1;
            joint_min.l_leg_kny = 0.2;
            joint_min.r_leg_kny = 0.2;
            obj.joint_min = double(joint_min);
          
            joint_max = Point(coords,[joint_max;0*joint_max]);
            joint_max.back_bky = 0.1;
            joint_max.back_bkx = 0.1;
            buffer=0.05;
            joint_max.l_leg_kny= joint_max.l_leg_kny-buffer;
            joint_max.r_leg_kny= joint_max.r_leg_kny-buffer;
            obj.joint_max = double(joint_max);
        end
        
        function [cache] = getPlanCache(obj)
            cache  = obj.plan_cache;
        end
        
        function setVDesired(obj,val)
            obj.plan_cache.v_desired  = val;
        end   

        function setQdotDesired(obj,val)
            obj.plan_cache.qdot_desired  = val;
        end       
                    
        
        function setHardwareMode(obj,mode)
            obj.hardware_mode = mode;
        end
        
        function val=isSimMode(obj)
            val = (obj.hardware_mode == 1);
        end
        
        function val=isBDIManipMode(obj)
            val = (obj.hardware_mode == 2);
        end
        
        function val=isBDIUserMode(obj)
            val = (obj.hardware_mode == 3);
        end

        function cachePelvisPose(obj,tspan,kc_name,pose)
            kc_pelvis = ActionKinematicConstraint(obj.r,obj.pelvis_body,[0;0;0],pose,tspan,kc_name);
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_pelvis);
        end
        function cacheLHandPose(obj,tspan,kc_name,pose)
            kc_lhand = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],pose,tspan,kc_name);
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lhand);
        end
        
        function cacheRHandPose(obj,tspan,kc_name,pose)
            kc_rhand = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],pose,tspan,kc_name);
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rhand);
        end
        
        function cacheLFootPose(obj,tspan,kc_name,l_foot_pose)
            kc_lfoot = ActionKinematicConstraint(obj.r,obj.l_foot_body,[0;0;0],l_foot_pose,tspan,kc_name);
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lfoot);
        end
        
        function cacheRFootPose(obj,tspan,kc_name,r_foot_pose)
            kc_rfoot = ActionKinematicConstraint(obj.r,obj.r_foot_body,[0;0;0],r_foot_pose,tspan,kc_name);
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rfoot);
        end
        
        function cacheLFootPoseAsContactConstraint(obj,tspan,kc_name,l_foot_pose)
            l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
            num_l_foot_pts = size(l_foot_pts,2);
            
            T_world_l_foot = [quat2rotmat(l_foot_pose(4:7)) l_foot_pose(1:3);0 0 0 1];
            l_foot_pts_pose = T_world_l_foot*[l_foot_pts;ones(1,num_l_foot_pts)];
            l_foot_pts_pose = [l_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_l_foot_pts),l_foot_pose(4:7))];
            lfoot_const.min = l_foot_pts_pose-1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
            lfoot_const.max = l_foot_pts_pose+1e-6*[ones(3,num_l_foot_pts);ones(4,num_l_foot_pts)];
            lfoot_const_static_contact = lfoot_const;
            lfoot_const_static_contact.contact_state = ...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)};
            
            kc_lfoot = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,lfoot_const_static_contact,tspan,kc_name,...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                {ContactAffordance()},...
                {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lfoot);
        end
        
        function cacheRFootPoseAsContactConstraint(obj,tspan,kc_name,r_foot_pose)
            r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
            num_r_foot_pts = size(r_foot_pts,2);
            
            T_world_r_foot = [quat2rotmat(r_foot_pose(4:7)) r_foot_pose(1:3);0 0 0 1];
            r_foot_pts_pose = T_world_r_foot*[r_foot_pts;ones(1,num_r_foot_pts)];
            r_foot_pts_pose = [r_foot_pts_pose(1:3,:); bsxfun(@times,ones(1,num_r_foot_pts),r_foot_pose(4:7))];
            rfoot_const.min = r_foot_pts_pose-1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
            rfoot_const.max = r_foot_pts_pose+1e-6*[ones(3,num_r_foot_pts);ones(4,num_r_foot_pts)];
            rfoot_const_static_contact = rfoot_const;
            rfoot_const_static_contact.contact_state = ...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)};
            kc_rfoot = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,rfoot_const_static_contact,tspan,kc_name,...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                {ContactAffordance()},...
                {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
            
            obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rfoot);
        end
        
        function replaceCachedConstraint(obj,kc_name,new_pose)
            if(sum(strcmp(obj.plan_cache.ks.kincon_name,kc_name))>0) %exists
                % delete old and add new
                obj.plan_cache.ks = deleteKinematicConstraint(obj.plan_cache.ks,kc_name);
            end
            s = [0 1];
            if(strcmp(kc_name,'lhandT'))
                kc_lhandT = ActionKinematicConstraint(obj.r,obj.l_hand_body,[0;0;0],new_pose,[s(end),s(end)],'lhandT');
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lhandT);
            elseif(strcmp(kc_name,'rhandT'))
                kc_rhandT = ActionKinematicConstraint(obj.r,obj.r_hand_body,[0;0;0],new_pose,[s(end),s(end)],'rhandT');
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rhandT);
            elseif(strcmp(kc_name,'headT'))
                kc_headT = ActionKinematicConstraint(obj.r, obj.head_body,[0;0;0],new_pose,[s(end),s(end)],'headT');
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_headT);
            elseif(strcmp(kc_name,'pelvis'))
                kc_pelvis = ActionKinematicConstraint(obj.r,obj.pelvis_body,[0;0;0],new_pose,[s(end),s(end)],'pelvis');
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_pelvis);
            elseif(strcmp(kc_name,'rfootT'))
                r_foot_pts = getContactPoints(getBody(obj.r,obj.r_foot_body));
                num_r_foot_pts = size(r_foot_pts,2);
                 
                kc_rfootT = ActionKinematicConstraint(obj.r,obj.r_foot_body,r_foot_pts,new_pose,[s(end),s(end)],'rfootT',...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_r_foot_pts)},...
                    {ContactAffordance()},...
                    {struct('max',zeros(1,num_r_foot_pts),'min',zeros(1,num_r_foot_pts))});
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_rfootT);
            elseif(strcmp(kc_name,'lfootT'))
                l_foot_pts = getContactPoints(getBody(obj.r,obj.l_foot_body));
                num_l_foot_pts = size(l_foot_pts,2);
                 
                kc_lfootT = ActionKinematicConstraint(obj.r,obj.l_foot_body,l_foot_pts,new_pose,[s(end),s(end)],'lfootT',...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,num_l_foot_pts)},...
                    {ContactAffordance()},...
                    {struct('max',zeros(1,num_l_foot_pts),'min',zeros(1,num_l_foot_pts))});
                obj.plan_cache.ks = obj.plan_cache.ks.addKinematicConstraint(kc_lfootT);
            end
        end
        
        function removeConstraintsContainingStr(obj,str)
                j=1;
               constraint_names= [];
               for i=1:length(obj.plan_cache.ks.kincon_name)
                   if(regexp(obj.plan_cache.ks.kincon_name{i},str)==1)
                      constraint_names{j} = obj.plan_cache.ks.kincon_name{i}; %accumulate
                      j=j+1;
                  end
               end
               % remove now
               for i=1:length(constraint_names)
                    obj.plan_cache.ks = deleteKinematicConstraint(obj.plan_cache.ks,constraint_names{i});
               end
              
        end
    end
    
end
