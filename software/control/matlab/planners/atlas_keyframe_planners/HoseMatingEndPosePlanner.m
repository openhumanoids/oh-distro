classdef HoseMatingEndPosePlanner < EndPosePlanner
  properties
    nozzle_base_radius
    nozzle_collar_radius
    nozzle_hand 
    hose_hand 
    T_hand_nozzle
    nozzle_axis
    nozzle_pose
    wye_pose
    affordance_listener
    T_palm_hand_nozzle; % Transformation from hand to palm for the hand grabbing the nozzle
    T_palm_hand_hose;
    hose_palm_pt
  end
  methods
    function obj = HoseMatingEndPosePlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode)
      obj = obj@EndPosePlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode);
      obj.pelvis_upright_gaze_tol = pi/20;
      obj.ee_torso_dist_lb = 0.5;
      obj.nozzle_hand = obj.r_hand_body;
      obj.hose_hand = obj.l_hand_body;
      obj.nozzle_axis = [0;0;1];
      obj.affordance_listener = AffordanceStateListener('AFFORDANCE_COLLECTION');
      saved_data = load('HoseMating.mat');
      obj.T_hand_nozzle = saved_data.T_hand_nozzle;
    end
    
    function [lfoot_constraint,rfoot_constraint,pelvis_constraint,head_constraint,dist_constraint,qsc,joint_constraint] ...
        = parseFixedConstraint(obj,x0)
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
      

      obj.updateNozzlePose();
      head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,obj.nozzle_pose(1:3),obj.h_camera_origin,obj.head_gaze_tol)};
      
      l_foot_pose = l_foot_pose0;
      r_foot_pose = r_foot_pose0;


      % left foot on ground
      lfoot_constraint ={WorldPositionConstraint(obj.r,obj.l_foot_body,l_foot_contact_pts,...
          [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)],...
          [nan(2,size(l_foot_contact_pts,2));l_foot_contact_pos(3,:)])};
      % right foot on ground
      rfoot_constraint = {WorldPositionConstraint(obj.r,obj.r_foot_body,r_foot_contact_pts,...
          [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)],...
          [nan(2,size(r_foot_contact_pts,2));r_foot_contact_pos(3,:)])};
      % pelvis upright
      pelvis_constraint = {WorldGazeDirConstraint(obj.r,obj.pelvis_body,[0;0;1],[0;0;1],obj.pelvis_upright_gaze_tol)};
            
      
      obj.updateWyePose();
      T_world_wye = HT(obj.wye_pose(1:3),obj.wye_pose(4),obj.wye_pose(5),obj.wye_pose(6));
      % pelvis facing the wye
      pelvis_constraint = [pelvis_constraint,{WorldPositionInFrameConstraint(obj.r,obj.pelvis_body,[0;0;0],T_world_wye,[-0.9;-0.4;-0.7],[-0.4;0.4;0.5])}];

      % distance between two feed
      % distance between hand and torso
      dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.nozzle_hand,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf),...
        Point2PointDistanceConstraint(obj.r,obj.hose_hand,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb/2,inf),...
        Point2PointDistanceConstraint(obj.r,obj.l_foot_body,obj.r_foot_body,[0;0;0],[0;0;0],0.3,inf)};
      
     
      qsc = QuasiStaticConstraint(obj.r);
      qsc = qsc.addContact(obj.r_foot_body,r_foot_contact_pts,obj.l_foot_body,l_foot_contact_pts);  
      qsc = qsc.setActive(true);
      qsc = qsc.setShrinkFactor(0.9); % search for a conservative pose
      
      
      joint_constraint = PostureConstraint(obj.r);
      joint_constraint = joint_constraint.setJointLimits((1:obj.r.getNumDOF)',obj.joint_constraint.lb,obj.joint_constraint.ub);
      coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
      back_z_ind = strcmp(coords,'back_bkz');
      joint_ind = (1:obj.r.getNumDOF)';
      l_leg_kny = joint_ind(strcmp(coords,'l_leg_kny'));
      r_leg_kny = joint_ind(strcmp(coords,'r_leg_kny'));
      joint_constraint = joint_constraint.setJointLimits(joint_ind(back_z_ind),-pi/6,pi/6);
      neck_idx = joint_ind(strcmp(coords,'neck_ay'));
      joint_constraint = joint_constraint.setJointLimits(neck_idx,0,0.3*pi);
      joint_constraint = joint_constraint.setJointLimits([l_leg_kny;r_leg_kny],[0.2*pi;0.2*pi],[inf;inf]);
    end
    
    function runPoseOptimizationViaMultitimeIKtraj(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,lidar_ee_goal,goal_type_flags)
      disp('Generating candidate hose mating endpose via IKtraj Given EE loci...')
      send_status(3,0,0,'Generating candidate endpose given EE Loci...');
      q0 = x0(1:getNumDOF(obj.r));
      [iktraj_lfoot_constraint,iktraj_rfoot_constraint,iktraj_pelvis_constraint,iktraj_head_constraint,iktraj_dist_constraint,qsc,joint_constraint] = ...
        obj.parseFixedConstraint(x0);
      
      
      
      iktraj_nozzle_hand_constraint = {};
      iktraj_hose_hand_constraint = {};
      tspan = [0 1];
      
      obj.updateNozzlePose();
      
      timeIndices = unique(Indices);

            
      lh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.lh_name)),ee_names));
      rh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.rh_name)),ee_names));
      lhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,lh_indices));
      rhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,rh_indices));
      if(isempty(lhand_constraints))
        updateNozzleHand(obj,obj.r_hand_body);
        nozzle_hand_constraints = rhand_constraints;
      elseif(isempty(rhand_constraints))
        updateNozzleHand(obj,obj.l_hand_body);
        nozzle_hand_constraints = lhand_constraints;
      end
      
      obj.updateWyePose();
      T_world_wye = HT(obj.wye_pose(1:3),obj.wye_pose(4),obj.wye_pose(5),obj.wye_pose(6));
      
      N = length(timeIndices);
      NBreaks = min(N,5);% No more than 5 breaks for IKTraj.
      s_breaks = linspace(0,1,NBreaks);
      s = linspace(0,1,N);
      
      iktraj_nozzle_hand_constraint = {};
      iktraj_hose_hand_constraint = {};
      
      if(obj.nozzle_hand == obj.r_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;-pi/3]) [0;0;0];0 0 0 1]; 
      elseif(obj.nozzle_hand == obj.l_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;pi/3]) [0;0;0];0 0 0 1]; 
      end
      iktraj_hose_hand_constraint = [iktraj_hose_hand_constraint,{WorldPositionInFrameConstraint(obj.r,obj.hose_hand,obj.hose_palm_pt,T_world_wye,[-0.5;-0.4;-0.4],[-0.2;0.4;0.2],tspan)}];
%       iktraj_hose_hand_constraint = [iktraj_hose_hand_constraint,{WorldFixedBodyPoseConstraint(obj.r,obj.hose_hand,tspan)}];
        
      for j = 1:NBreaks
        si = s_breaks(j);
        nozzle_hand_pose = pose_spline(s,nozzle_hand_constraints,si);
        T_world_palm_nozzle = [quat2rotmat(nozzle_hand_pose(4:7)) nozzle_hand_pose(1:3);0 0 0 1];
        T_world_hand_nozzle = T_world_palm_nozzle*obj.T_palm_hand_nozzle;
        nozzle_hand_pose = [T_world_hand_nozzle(1:3,4);rotmat2quat(T_world_hand_nozzle(1:3,1:3))];
        iktraj_nozzle_hand_constraint = [iktraj_nozzle_hand_constraint,parse2PosQuatConstraint(obj.r,obj.nozzle_hand,[0;0;0],nozzle_hand_pose,2e-2,sind(6)^2,[si,si])];
      end
      
      
      cost = getCostVector(obj);
      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setDebug(true);
      iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
      iktraj_options = iktraj_options.setQa(0.0005*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1));
      iktraj_options = iktraj_options.setFixInitialState(false);
      iktraj_options = iktraj_options.setMajorIterationsLimit(1000);
      iktraj_options = iktraj_options.setIterationsLimit(50000);

      
      iktraj_options = iktraj_options.setMajorIterationsLimit(1000);
      nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      qstar = nomdata.xstar(1:obj.r.getNumDOF());
      iktraj_tbreaks = s_breaks;
      iktraj_qseed_traj = PPTrajectory(foh(iktraj_tbreaks,[q0 repmat(qstar,1,NBreaks-1)]));
      iktraj_qnom_traj = PPTrajectory(foh(iktraj_tbreaks,repmat(qstar,1,NBreaks)));           

     
      coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
     
      joint_ind = (1:obj.r.getNumDOF)';
      lower_joint_idx = joint_ind(cellfun(@(s) ~isempty(strfind(s,'leg')),coords));
      pc_fixed = PostureChangeConstraint(obj.r,[(1:6)';lower_joint_idx],zeros(6+length(lower_joint_idx),1),zeros(6+length(lower_joint_idx),1));
          
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        iktraj_tbreaks,iktraj_qseed_traj,iktraj_qnom_traj,...
        iktraj_rfoot_constraint{:},iktraj_lfoot_constraint{:},...
        iktraj_pelvis_constraint{:},iktraj_head_constraint{:},...
        joint_constraint,iktraj_hose_hand_constraint{:},iktraj_nozzle_hand_constraint{:},...
        qsc,iktraj_dist_constraint{:},...
        pc_fixed,...
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
      qdot0 = x_breaks(obj.r.getNumDOF+(1:obj.r.getNumDOF),1);
      qdotf = x_breaks(obj.r.getNumDOF+(1:obj.r.getNumDOF),end);
      qtraj = PPTrajectory(spline(s_breaks,[qdot0 q_breaks qdotf]));

      s = linspace(0,1,10);
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      xtraj_atlas = zeros(2*nq_atlas,1);
      disp('Publishing candidate endpose for ee_loci ...');
      send_status(3,0,0,'Publishing candidate endpose for ee_loci...');

      for j = 1:length(s),

        utime = get_timestamp_now();
        q_tmp = qtraj.eval(s(j));
        xtraj_atlas(1:nq_atlas) = q_tmp(obj.atlas2robotFrameIndMap(1:nq_atlas));
        obj.pose_pub.publish(xtraj_atlas,utime);
        pause(0.1);
      end
          
    end
    
    
    
    
    function runPoseOptimizationViaSingleTimeIK(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,lidar_ee_goal,goal_type_flags)
      disp('Generating candidate hose mating endpose via IK Given EE loci...')
      send_status(3,0,0,'Generating candidate hose mating endpose given EE Loci...');
      q0 = x0(1:getNumDOF(obj.r));
      [lfoot_constraint,rfoot_constraint,pelvis_constraint,head_constraint,dist_constraint,qsc,joint_constraint] = ...
        obj.parseFixedConstraint(x0);
      
        
      nozzle_hand_constraint = {};
      hose_hand_constraint = {};
      
      
      obj.updateNozzlePose();
      
      timeIndices = unique(Indices);

      ind=find(Indices==timeIndices(1));   
      lhandT = [];
      rhandT = [];
      for k = 1:length(ind)
        if(strcmp(obj.lh_name,ee_names{ind(k)}))
          lh_ee_goal = ee_loci(:,ind(k));
          lhandT = zeros(6,1);
          T_world_palm_l = HT(lh_ee_goal(1:3),lh_ee_goal(4),lh_ee_goal(5),lh_ee_goal(6));
          T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
          lhandT(1:3) = T_world_hand_l(1:3,4);
          lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
          l_hand_pose = [lhandT(1:3); rpy2quat(lhandT(4:6))];
        elseif(strcmp(obj.rh_name,ee_names{ind(k)}))
          rh_ee_goal = ee_loci(:,ind(k));
          rhandT = zeros(6,1);
          T_world_palm_r = HT(rh_ee_goal(1:3),rh_ee_goal(4),rh_ee_goal(5),rh_ee_goal(6));
          T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
          rhandT(1:3) = T_world_hand_r(1:3,4);
          rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
          r_hand_pose = [rhandT(1:3); rpy2quat(rhandT(4:6))];
        end
      end

      if(isempty(lhandT))
        updateNozzleHand(obj,obj.r_hand_body);
        nozzle_hand_pose = r_hand_pose;
      elseif(isempty(rhandT))
        updateNozzleHand(obj,obj.l_hand_body);
        nozzle_hand_pose = l_hand_pose;
      end
      
      obj.updateWyePose();
      T_world_wye = HT(obj.wye_pose(1:3),obj.wye_pose(4),obj.wye_pose(5),obj.wye_pose(6));
      
      
      nozzle_hand_constraint = {};
      hose_hand_constraint = {};
      
      if(obj.nozzle_hand == obj.r_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;-pi/3]) [0;0;0];0 0 0 1]; 
      elseif(obj.nozzle_hand == obj.l_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;pi/3]) [0;0;0];0 0 0 1]; 
      end
      hose_hand_constraint = [hose_hand_constraint,{WorldPositionInFrameConstraint(obj.r,obj.hose_hand,obj.hose_palm_pt,T_world_wye,[-0.5;-0.4;-0.4],[-0.2;0.4;0.2])}];
%       iktraj_hose_hand_constraint = [iktraj_hose_hand_constraint,{WorldFixedBodyPoseConstraint(obj.r,obj.hose_hand,tspan)}];
        
      
      
      
      T_world_palm_nozzle = [quat2rotmat(nozzle_hand_pose(4:7)) nozzle_hand_pose(1:3);0 0 0 1];
      T_world_hand_nozzle = T_world_palm_nozzle*obj.T_palm_hand_nozzle;
      nozzle_hand_pose = [T_world_hand_nozzle(1:3,4);rotmat2quat(T_world_hand_nozzle(1:3,1:3))];
      nozzle_hand_constraint = [nozzle_hand_constraint,parse2PosQuatConstraint(obj.r,obj.nozzle_hand,[0;0;0],nozzle_hand_pose,2e-2,sind(4)^2)];
      

      cost = getCostVector(obj);
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setQ(diag(cost(1:getNumDOF(obj.r))));
      ikoptions = ikoptions.setDebug(true);

      qseed = q0;
      nomdata = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      qnom = nomdata.xstar(1:obj.r.getNumDOF());
      [q,snopt_info,infeasible_constraint] = inverseKin(obj.r,...
        qseed,qnom,...
        rfoot_constraint{:},lfoot_constraint{:},...
        pelvis_constraint{:},head_constraint{:},...
        joint_constraint,hose_hand_constraint{:},nozzle_hand_constraint{:},...
        qsc,dist_constraint{:},...
        ikoptions);
      if(snopt_info > 10)
          warning('poseOpt IK fails');
          send_msg = sprintf('snopt_info = %d. endpose IK fails.',snopt_info);
          send_status(4,0,0,send_msg);
          display(infeasibleConstraintMsg(infeasible_constraint));
      end
      
       disp('Publishing candidate endpose ...');
       send_status(3,0,0,'Publishing candidate endpose...');
       utime = get_timestamp_now();% equivalent to bot_timestamp_now();
       nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
       xtraj_atlas = zeros(2*nq_atlas,1);
       xtraj_atlas(1:nq_atlas,:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
       obj.pose_pub.publish(xtraj_atlas,utime);
    end  
        
        
        
    
    function updateNozzleHandTransform(obj,q)
      obj.updateNozzlePose();
      kinsol = obj.r.doKinematics(q);
      T_world_nozzle = HT(obj.nozzle_pose(1:3),obj.nozzle_pose(4),obj.nozzle_pose(5),obj.nozzle_pose(6));
      nozzle_hand_pose = forwardKin(obj.r,kinsol,obj.nozzle_hand,[0;0;0],2);
      T_world_hand = [quat2rotmat(nozzle_hand_pose(4:7)) nozzle_hand_pose(1:3);0 0 0 1];
      obj.T_hand_nozzle = inv_HT(T_world_hand)*T_world_nozzle;
    end
    
    function updateNozzleHand(obj,nozzle_hand)
      if(nozzle_hand == obj.l_hand_body)
        obj.nozzle_hand = nozzle_hand;
        obj.hose_hand = obj.r_hand_body;
        obj.T_palm_hand_nozzle = obj.T_palm_hand_l;
        obj.T_palm_hand_hose = obj.T_palm_hand_r;
        obj.hose_palm_pt =[0;-0.15;0];
      elseif(nozzle_hand == obj.r_hand_body)
        obj.nozzle_hand = nozzle_hand;
        obj.hose_hand = obj.l_hand_body;
        obj.T_palm_hand_nozzle = obj.T_palm_hand_r;
        obj.T_palm_hand_hose = obj.T_palm_hand_l;
        obj.hose_palm_pt = [0;0.15;0];
      else
        error('The hand body index is incorrect');
      end
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
    
    function updateNozzlePose(obj)
      receive_aff_flag = false;
      obj.nozzle_pose = [];
      while(~receive_aff_flag)
        data = obj.affordance_listener.getNextMessage(5);
        if(~isempty(data))
          for i = 1:obj.affordance_listener.naffs
            if(strcmp(data.otdf_type{i},'firehose_simple'))
              obj.nozzle_pose = [data.xyz(:,i);data.rpy(:,i)];
            end
          end
          if(isempty(obj.nozzle_pose))
            error('nozzle is not in the affordance collection LCM message');
          end
          receive_aff_flag = true;
        end
      end
    end
    
    
  end
end
