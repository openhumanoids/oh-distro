classdef HoseMatingReachingPlanner < ReachingPlanner
  properties
    affordance_listener
    wye_pose;
    nozzle_hand;
  end
  
  methods
    function obj = HoseMatingReachingPlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode)
      obj = obj@ReachingPlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode);
      obj.affordance_listener = AffordanceStateListener('AFFORDANCE_COLLECTION');
      obj.nozzle_hand = obj.l_hand_body;
    end
    
    function [xtraj_atlas,snopt_info] = getPlan(obj,x0,mode)
      % mode = drc.hose_mating_cmd_t.hose_align  Align the elbow
      % mode = drc.hose_mating_cmd_t.hose_insert Insert the hose
      display('Generating reaching plan for hose mating');
      obj.planning_mode = 4;
      obj.plan_cache.clearCache();
      obj.plan_cache.num_breaks = obj.num_breaks;
      q0 = x0(1:obj.r.getNumDOF);
      obj.updateWyePose();
      T_world_wye = HT(obj.wye_pose(1:3),obj.wye_pose(4),obj.wye_pose(5),obj.wye_pose(6));
      head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,obj.wye_pose(1:3),obj.h_camera_origin,obj.head_gaze_tol)};
      
      if(obj.nozzle_hand == obj.r_hand_body)
        nozzle_farm_axis = [0;-1;0];
        nozzle_farm = obj.r.findLinkInd('r_farm');
        wye_mate_pt = [-0.009;-0.066;0.004];
        wye_mate_axis = rpy2rotmat([0;0;pi/2-1])*[1;0;0];
        T_wye_wye_axis = HT(wye_mate_pt,0,0,pi/2-1);
      elseif(obj.nozzle_hand == obj.l_hand_body)
        nozzle_farm_axis = [0;1;0];
        nozzle_farm = obj.r.findLinkInd('l_farm');
        wye_mate_pt = [-0.009;0.066;0.004]; % The center of the cylinder on the wye, to be mated
        wye_mate_axis = rpy2rotmat([0;0;-(pi/2-1)])*[1;0;0];
        T_wye_wye_axis = HT(wye_mate_pt,0,0,-(pi/2-1));
      end
      wye_axis_world = T_world_wye*[[wye_mate_pt;1] [wye_mate_pt+wye_mate_axis;1]];
      wye_axis_world = [wye_axis_world(1,2)-wye_axis_world(1,1);wye_axis_world(2,2)-wye_axis_world(2,1);wye_axis_world(3,2)-wye_axis_world(3,1)];
      
      final_farm_wye_dist = -0.4;
      if(mode == drc.hose_mating_cmd_t.hose_align)
        align_tspan = [1,1];
      elseif(mode == drc.hose_mating_cmd_t.hose_insert)
        align_tspan = [0,1];
      end
      nozzle_hand_constraint = {WorldGazeDirConstraint(obj.r,nozzle_farm,nozzle_farm_axis,wye_axis_world,0,align_tspan)};
      if(mode == drc.hose_mating_cmd_t.hose_align)
        nozzle_hand_constraintT = {WorldPositionInFrameConstraint(obj.r,nozzle_farm,[0;0;0],T_world_wye*T_wye_wye_axis,[final_farm_wye_dist-0.05;0;0],[final_farm_wye_dist-0.02;0;0],[1,1])};
        nozzle_hand_constraint = [nozzle_hand_constraint,nozzle_hand_constraintT];
      elseif(mode == drc.hose_mating_cmd_t.hose_insert)
        nozzle_hand_constraint = [nozzle_hand_constraint,{WorldPositionInFrameConstraint(obj.r,nozzle_farm,[0;0;0],T_world_wye*T_wye_wye_axis,[final_farm_wye_dist-0.05;-0.005;-0.005],[final_farm_wye_dist;0.005;0.005],align_tspan)}];
        nozzle_hand_constraintT = {WorldPositionInFrameConstraint(obj.r,nozzle_farm,[0;0;0],T_world_wye*T_wye_wye_axis,[final_farm_wye_dist;0;0],[final_farm_wye_dist;0;0],[1,1])};
        nozzle_hand_constraint = [nozzle_hand_constraint,nozzle_hand_constraintT];
      end
      
      joint_constraint = PostureConstraint(obj.r);
      joint_constraint = joint_constraint.setJointLimits(obj.lower_joint_ind,q0(obj.lower_joint_ind),q0(obj.lower_joint_ind));
      if(obj.nozzle_hand == obj.r_hand_body)
        joint_constraint = joint_constraint.setJointLimits(obj.l_arm_joint_ind,q0(obj.l_arm_joint_ind),q0(obj.l_arm_joint_ind));
      elseif(obj.nozzle_hand == obj.l_hand_body)
        joint_constraint = joint_constraint.setJointLimits(obj.r_arm_joint_ind,q0(obj.r_arm_joint_ind),q0(obj.r_arm_joint_ind));
      end
      
      ikoptions = IKoptions(obj.r);
      [q_final_guess,info] = inverseKin(obj.r,q0,q0,nozzle_hand_constraintT{:},head_constraint{:},joint_constraint,ikoptions);
      
      qtraj_guess = PPTrajectory(spline([0 1],[zeros(obj.r.getNumDOF,1) q0 q_final_guess zeros(obj.r.getNumDOF,1)]));
      cost = getCostVector(obj);
      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
      iktraj_options = iktraj_options.setQa(0.05*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1)); % upper and lower bnd on velocity.
      iktraj_options = iktraj_options.setMajorIterationsLimit(500);
      iktraj_options = iktraj_options.setDebug(true);
%       iktraj_options = iktraj_options.setAdditionaltSamples(linspace(0,1,5));
      iktraj_tbreaks = linspace(0,1,4);
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,iktraj_tbreaks,qtraj_guess,qtraj_guess,nozzle_hand_constraint{:},head_constraint{:},joint_constraint,iktraj_options);
      
      if(snopt_info>10)
        send_msg = sprintf('SNOPT_INFO = %d',snopt_info);
        msg_color = 4;
      else
        send_msg = sprintf('IKtraj succeeds');
        msg_color = 2;
      end
      send_status(msg_color,0,0,send_msg);
      
      s_breaks = iktraj_tbreaks;
      x_breaks = xtraj.eval(s_breaks);
      q_breaks = x_breaks(1:obj.r.getNumDOF,:);
      qtraj_guess = PPTrajectory(spline(s_breaks,[zeros(obj.r.getNumDOF,1) q_breaks zeros(obj.r.getNumDOF,1)]));
      
      Tmax_ee=obj.getTMaxForMaxEEArcSpeed(s_breaks,q_breaks);
      s_total = Tmax_ee*obj.plan_cache.v_desired;
      s = linspace(0,1,max(ceil(s_total/obj.plan_arc_res)+1,5)); % Must have two points atleast
      s = unique([s(:);s_breaks(:)]);
      s = s(:)';

      q = qtraj_guess.eval(s);
      q(:,1) = q_breaks(:,1);
      qdot = qtraj_guess.deriv(s);
      qdot0 = qdot(:,1);
      qdotf = qdot(:,end);
     
      obj.plan_cache.lhand_constraint_cell = nozzle_hand_constraint;
     
      obj.plan_cache.s = s;
      obj.plan_cache.s_breaks = s_breaks;
      obj.plan_cache.qtraj = PPTrajectory(spline(s, [qdot0 q qdotf]));
      
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      xtraj_atlas = zeros(2+2*nq_atlas,length(s));

      xtraj_atlas(1,:) = 0*s;
      xtraj_atlas(2,:) = 0*s;
      if(length(s_breaks)>obj.plan_cache.num_breaks)
          keyframe_inds = unique(round(linspace(1,length(s_breaks),obj.plan_cache.num_breaks)));
      else
          keyframe_inds = 1:length(s_breaks);
      end

      for l = keyframe_inds,
          xtraj_atlas(1,s == s_breaks(l)) = 1.0;
      end
      xtraj_atlas(2+(1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
      xtraj_atlas(2+nq_atlas+(1:nq_atlas),:) = qdot(obj.atlas2robotFrameIndMap(nq_atlas+(1:nq_atlas))-size(q,1),:);
      snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));

      Tmax_joints=obj.getTMaxForMaxJointSpeed();
      ts = s.*max(Tmax_joints,Tmax_ee); % plan timesteps

      obj.plan_cache.time_2_index_scale = 1./(max(Tmax_joints,Tmax_ee));
      utime = get_timestamp_now();% equivalent to bot_timestamp_now();

      obj.plan_pub.publish(xtraj_atlas,ts,utime, snopt_info_vector);
      display(sprintf('Reaching planner ts %5.3f\n',ts(end)));
      planner_data = obj.checkPlannerConfig(ts(end)-ts(1));
      obj.planner_config_publisher.publish(utime,planner_data);
    end
    
    function updateWyePose(obj)
      receive_aff_flag = false;
      obj.wye_pose = [];
      while(~receive_aff_flag)
        data = obj.affordance_listener.getNextMessage(5);
        if(~isempty(data))
          for i = 1:obj.affordance_listener.naffs
            if(strcmp(data.otdf_type{i},'wye'))
              obj.wye_pose = [data.xyz(:,i);data.rpy(:,i)];
            end
          end
          if(isempty(obj.wye_pose))
            error('wye is not in the affordance collection LCM message');
          end
          receive_aff_flag = true;
        end
      end
    end
  end
end