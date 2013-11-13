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
    T_palm_hand_nozzle;
    T_palm_hand_hose;
    hose_palm_pt
  end
  methods
    function obj = HoseMatingEndPosePlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode)
      obj = obj@EndPosePlanner(r,atlas,lhand_frame,rhand_frame,hardware_mode);
      obj.pelvis_upright_gaze_tol = pi/30;
      obj.ee_torso_dist_lb = 0.5;
      obj.nozzle_hand = obj.r_hand_body;
      obj.hose_hand = obj.l_hand_body;
      obj.nozzle_axis = [0;0;1];
      obj.affordance_listener = AffordanceStateListener('AFFORDANCE_COLLECTION');
      
    end
    
    function runPoseOptimizationViaMultitimeIKtraj(obj,x0,ee_names,ee_loci,Indices,rh_ee_goal,lh_ee_goal,h_ee_goal,goal_type_flags)
      disp('Generating candidate endpose via IKtraj Given EE loci...')
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
      
      iktraj_nozzle_hand_constraint = {};
      iktraj_hose_hand_constraint = {};
      tspan = [0 1];
      if(goal_type_flags.rh == 2)
          iktraj_rhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.r_hand_body,obj.rh_gaze_axis,rh_ee_goal(1:3),obj.rh_camera_origin,obj.hand_gaze_tol)};
      else
          iktraj_rhand_constraint = {};
      end
      if(goal_type_flags.lh == 2)
          iktraj_lhand_constraint = {WorldGazeTargetConstraint(obj.r,obj.l_hand_body,obj.lh_gaze_axis,lh_ee_goal(1:3),obj.lh_camera_origin,obj.hand_gaze_tol)};
      else
          iktraj_lhand_constraint = {};
      end
%       if(goal_type_flags.h == 2)
%           iktraj_head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,h_ee_goal(1:3),obj.h_camera_origin,obj.head_gaze_tol,tspan)};
%       else
%           iktraj_head_constraint = {};
%       end
      obj.updateNozzlePose();
      iktraj_head_constraint = {WorldGazeTargetConstraint(obj.r,obj.head_body,obj.head_gaze_axis,obj.nozzle_pose(1:3),obj.h_camera_origin,obj.head_gaze_tol,tspan)};
      
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

      lfoot_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.l_foot_body,tspan)};
      rfoot_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.r_foot_body,tspan)};
      iktraj_lfoot_constraint = [iktraj_lfoot_constraint,lfoot_constraint];     
      iktraj_rfoot_constraint = [iktraj_rfoot_constraint,rfoot_constraint];
      iktraj_pelvis_constraint = {WorldFixedBodyPoseConstraint(obj.r,obj.pelvis_body,tspan),...
        WorldGazeDirConstraint(obj.r,obj.pelvis_body,[0;0;1],[0;0;1],obj.pelvis_upright_gaze_tol,tspan)};
            
      
      timeIndices = unique(Indices);

            
      lh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.lh_name)),ee_names));
      rh_indices = (~cellfun(@(x) isempty(strfind(char(x),obj.rh_name)),ee_names));
      lhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,lh_indices));
      rhand_constraints=convertEELociFromRPYToQuat(obj,ee_loci(:,rh_indices));
      if(isempty(lhand_constraints))
        updateNozzleHand(obj,obj.r_hand_body);
        nozzle_hand_constraints = rhand_constraints;
        hose_hand_constraints = [];
      elseif(isempty(rhand_constraints))
        updateNozzleHand(obj,obj.l_hand_body);
        nozzle_hand_constraints = lhand_constraints;
        hose_hand_constraints = [];
      end
      
      
      N = length(timeIndices);
      NBreaks = min(N,5);% No more than 5 breaks for IKTraj.
      s_breaks = linspace(0,1,NBreaks);
      s = linspace(0,1,N);
      
      iktraj_nozzle_hand_constraint = {};
      iktraj_hose_hand_constraint = {};
      obj.updateWyePose();
      T_world_wye = HT(obj.wye_pose(1:3),obj.wye_pose(4),obj.wye_pose(5),obj.wye_pose(6));
      if(obj.nozzle_hand == obj.r_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;-pi/3]) [0;0;0];0 0 0 1]; 
      elseif(obj.nozzle_hand == obj.l_hand_body)
        T_world_hose_mate = T_world_wye*[rpy2rotmat([0;0;pi/3]) [0;0;0];0 0 0 1]; 
      end
      iktraj_hose_hand_constraint = [iktraj_hose_hand_constraint,{WorldPositionInFrameConstraint(obj.r,obj.hose_hand,obj.hose_palm_pt,T_world_hose_mate,[-0.5;-0.2;-0.2],[-0.2;0.2;0.1],tspan)}];
%       iktraj_hose_hand_constraint = [iktraj_hose_hand_constraint,{WorldFixedBodyPoseConstraint(obj.r,obj.hose_hand,tspan)}];
        
      for j = 2:NBreaks
        si = s_breaks(j);
        nozzle_hand_pose = pose_spline(s,nozzle_hand_constraints,si);
        T_world_palm_nozzle = [quat2rotmat(nozzle_hand_pose(4:7)) nozzle_hand_pose(1:3);0 0 0 1];
        T_world_hand_nozzle = T_world_palm_nozzle*obj.T_palm_hand_nozzle;
        nozzle_hand_pose = [T_world_hand_nozzle(1:3,4);rotmat2quat(T_world_hand_nozzle(1:3,1:3))];
        iktraj_nozzle_hand_constraint = [iktraj_nozzle_hand_constraint,parse2PosQuatConstraint(obj.r,obj.nozzle_hand,[0;0;0],nozzle_hand_pose,2e-2,sind(4)^2,[si,si])];
      end
      
      dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.r_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,[0 1])};
      iktraj_dist_constraint = [iktraj_dist_constraint,dist_constraint];       
                        
      dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_hand_body,obj.utorso_body,[0;0;0],[0;0;0],obj.ee_torso_dist_lb,inf,[0 1])};
      iktraj_dist_constraint = [iktraj_dist_constraint,dist_constraint];   
      
      dist_constraint = {Point2PointDistanceConstraint(obj.r,obj.l_foot_body,obj.r_foot_body,[0;0;0],[0;0;0],0.3,inf,[0 1])};
      iktraj_dist_constraint = [iktraj_dist_constraint,dist_constraint];  
      
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
      coords = obj.r.getStateFrame.coordinates(1:obj.r.getNumDOF);
      back_z_ind = strcmp(coords,'back_bkz');
      joint_ind = (1:obj.r.getNumDOF)';
      joint_constraint = joint_constraint.setJointLimits(joint_ind(back_z_ind),-pi/6,pi/6);
          
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        iktraj_tbreaks,iktraj_qseed_traj,iktraj_qnom_traj,...
        iktraj_rhand_constraint{:},iktraj_lhand_constraint{:},...
        iktraj_rfoot_constraint{:},iktraj_lfoot_constraint{:},...
        iktraj_pelvis_constraint{:},iktraj_head_constraint{:},...
        joint_constraint,iktraj_hose_hand_constraint{:},iktraj_nozzle_hand_constraint{:},...
        qsc,iktraj_dist_constraint{:},...
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

        utime = now() * 24 * 60 * 60;
        q_tmp = qtraj.eval(s(j));
        xtraj_atlas(1:nq_atlas) = q_tmp(obj.atlas2robotFrameIndMap(1:nq_atlas));
        obj.pose_pub.publish(xtraj_atlas,utime);
        pause(0.1);
      end
          
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