classdef ManipulationPlanner < handle
  
  properties
    num_breaks
    s_breaks
    q_breaks
    qdos_breaks
    plan_pub  
    r 
    lhandT % cache goals
    rhandT
    qtraj_guess % old coarse keyframe sequence search
    qtraj_guess_fine % fine manip plan with individual IK fill ins
    time_2_index_scale
  end
  
  methods
    function obj = ManipulationPlanner(r)
      obj.r = r;
      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      obj.num_breaks = 4;
      obj.plan_pub = RobotPlanPublisherWKeyFrames('atlas',joint_names,true,'CANDIDATE_MANIP_PLAN',obj.num_breaks);
    end
    
    function adjustAndPublishManipulationPlan(obj,x0,r_ee_constraint,l_ee_constraint)       
        is_keyframe_constraint = true;
        runOptimization(obj,x0,r_ee_constraint,l_ee_constraint,is_keyframe_constraint);        
    end
    
    function generateAndPublishManipulationPlan(obj,x0,r_ee_goal,l_ee_goal)       
        is_keyframe_constraint = false;
        runOptimization(obj,x0,r_ee_goal,l_ee_goal,is_keyframe_constraint);        
    end
       
    function runOptimization(obj,x0,r_ee_goal,l_ee_goal,is_keyframe_constraint)
      disp('Generating plan...');

      %Replace time formulation with an Arc length formulation.
      
      
      
      q0 = x0(1:getNumDOF(obj.r)); 
      
      T_world_body = HT(x0(1:3),x0(4),x0(5),x0(6));
      %print_angles(T_world_body,'T_world_body');
      
      % get foot positions
      kinsol = doKinematics(obj.r,q0);
      rfoot_body = obj.r.findLink('r_foot');
      lfoot_body = obj.r.findLink('l_foot');    
       
      %   rfoot0 = forwardKin(obj.r,kinsol,rfoot_body,[0;0;0],1);
      %   lfoot0 = forwardKin(obj.r,kinsol,lfoot_body,[0;0;0],1);
      r_foot_pose0 = forwardKin(obj.r,kinsol,rfoot_body,...
      rfoot_body.contact_pts(:,[rfoot_body.collision_group{1}]),2); 
      r_foot_pose0 = mean(r_foot_pose0,2);

      l_foot_pose0 = forwardKin(obj.r,kinsol,lfoot_body,...
      lfoot_body.contact_pts(:,[lfoot_body.collision_group{1}]),2);
      l_foot_pose0 = mean(l_foot_pose0,2);
      
      % compute fixed COM goal
      gc = contactPositions(obj.r,q0);
      k = convhull(gc(1:2,:)');
      com0 = getCOM(obj.r,q0);
      %   comgoal = [mean(gc(1:2,k),2);com0(3)];
      %   comgoal = com0; % DOnt move com for now as this is pinned manipulation      
      
      r_hand_body = findLink(obj.r,'r_hand');
      l_hand_body = findLink(obj.r,'l_hand');

      % compute EE trajectories
      r_hand_pose0 = forwardKin(obj.r,kinsol,r_hand_body,[0;0;0],2);
      l_hand_pose0 = forwardKin(obj.r,kinsol,l_hand_body,[0;0;0],2);

      % Goals are presented in palm frame, must be transformed to hand coordinate frame
      % Using notation similar to KDL.
      % fixed transform between hand and palm as specified in the urdf
      T_hand_palm_l = HT([0;0.1;0],1.57079,0,1.57079);
      T_palm_hand_l = inv_HT(T_hand_palm_l);
      T_hand_palm_r = HT([0;-0.1;0],-1.57079,0,-1.57079);
      T_palm_hand_r = inv_HT(T_hand_palm_r);
      
      if(~is_keyframe_constraint)
          if(isempty(r_ee_goal))
              r_ee_goal = forwardKin(obj.r,kinsol,r_hand_body,[0;0;0],1);
              rhandT  = r_ee_goal(1:6);
              rhandT = [nan;nan;nan;nan;nan;nan];
          else
              rhandT = zeros(6,1);
              % Desired position of palm in world frame
              T_world_palm_r = HT(r_ee_goal(1:3),r_ee_goal(4),r_ee_goal(5),r_ee_goal(6));
              T_world_hand_r = T_world_palm_r*T_palm_hand_r;
              % print_angles(T_world_palm_r,'T_world_palm_r');
              % print_angles(T_world_hand_r,'T_world_hand_r');
              rhandT(1:3) = T_world_hand_r(1:3,4);
              rhandT(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
          end
          
          if(isempty(l_ee_goal))
              l_ee_goal = forwardKin(obj.r,kinsol,l_hand_body,[0;0;0],1);
              lhandT  = l_ee_goal(1:6);
              lhandT = [nan;nan;nan;nan;nan;nan];
          else
              lhandT = zeros(6,1);
              % Desired position of palm in world frame
              T_world_palm_l = HT(l_ee_goal(1:3),l_ee_goal(4),l_ee_goal(5),l_ee_goal(6));
              T_world_hand_l = T_world_palm_l*T_palm_hand_l;
              % print_angles(T_world_palm_l,'T_world_palm_l');
              %  print_angles(T_world_hand_l,'T_world_hand_l');
              lhandT(1:3) = T_world_hand_l(1:3,4);
              lhandT(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));
          end
          r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
          l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];   
          obj.rhandT = rhandT;
          obj.lhandT = lhandT;
      else
          
          rhandT =  obj.rhandT;
          lhandT =  obj.lhandT;
          r_hand_poseT = [rhandT(1:3); rpy2quat(rhandT(4:6))];
          l_hand_poseT = [lhandT(1:3); rpy2quat(lhandT(4:6))];   
          
          if(isempty(r_ee_goal))
              s_int_r= nan;
              rhand_int_constraint = [nan;nan;nan;nan;nan;nan];
          else
              rhand_int_constraint = zeros(6,1);
              s_int_r = r_ee_goal.time*obj.time_2_index_scale;
              % Desired position of palm in world frame
              rpy = quat2rpy(r_ee_goal.desired_pose(4:7));
              T_world_palm_r = HT(r_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
              T_world_hand_r = T_world_palm_r*T_palm_hand_r;
              rhand_int_constraint(1:3) = T_world_hand_r(1:3,4);
              rhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_r(1:3,1:3));
              
          end
          
          if(isempty(l_ee_goal))
              s_int_l = nan;
              lhand_int_constraint = [nan;nan;nan;nan;nan;nan];
          else
              lhand_int_constraint = zeros(6,1);
              s_int_l = l_ee_goal.time*obj.time_2_index_scale;
              % Desired position of palm in world frame
              rpy = quat2rpy(l_ee_goal.desired_pose(4:7));
              T_world_palm_l = HT(l_ee_goal.desired_pose(1:3),rpy(1),rpy(2),rpy(3));
              T_world_hand_l = T_world_palm_l*T_palm_hand_l;
              lhand_int_constraint(1:3) = T_world_hand_l(1:3,4);
              lhand_int_constraint(4:6) =rotmat2rpy(T_world_hand_l(1:3,1:3));     
        
          end
           r_hand_pose_int = [rhand_int_constraint(1:3); rpy2quat(rhand_int_constraint(4:6))];   
           l_hand_pose_int = [lhand_int_constraint(1:3); rpy2quat(lhand_int_constraint(4:6))];
          
      end
      
      
     
      %lhandT(1:3) = [nan;nan;nan];lhandT(4:6) = [0*(pi/180); 0*(pi/180);0*(pi/180)];
      %rhandT(1:3) = [nan;nan;nan];%rhandT(4:6) = [-pi/2;0;0-pi/2];
 
       
      ind = getActuatedJoints(obj.r);
      cost = getCostVector(obj);
      
      ikoptions = struct();
      ikoptions.Q = diag(cost(1:getNumDOF(obj.r)));
      ikoptions.q_nom = q0;
      if(is_keyframe_constraint)
         ikoptions.MajorIterationsLimit = 10; 
      else
         ikoptions.MajorIterationsLimit = 50;
      end
      dofnum =  find(ismember(obj.r.getStateFrame.coordinates,'l_arm_elx')==1);
      ikoptions.q_nom(dofnum) = 30*(pi/180);
      dofnum =  find(ismember(obj.r.getStateFrame.coordinates,'r_arm_elx')==1);
      ikoptions.q_nom(dofnum) = 30*(pi/180);
      comgoal.min = [com0(1)-.1;com0(2)-.1;com0(3)-.5];
      comgoal.max = [com0(1)+.1;com0(2)+.1;com0(3)+0.5];
      pelvis_body = findLink(obj.r,'pelvis'); % dont move pelvis
      pelvis_pose0 = forwardKin(obj.r,kinsol,pelvis_body,[0;0;0],2);
       
      
      
      s = [0 1]; % normalized arc length index 
      ks = ActionSequence();
      kc_com = ActionKinematicConstraint(obj.r,0,[0;0;0],comgoal,[s(1),s(end)],'com');
      ks = ks.addKinematicConstraint(kc_com);
      kc_rfoot = ActionKinematicConstraint(obj.r,rfoot_body,mean(rfoot_body.getContactPoints('heel'),2),r_foot_pose0,[s(1),s(end)],'rfoot_heel');
      ks = ks.addKinematicConstraint(kc_rfoot);
      kc_lfoot = ActionKinematicConstraint(obj.r,lfoot_body,mean(lfoot_body.getContactPoints('heel'),2),l_foot_pose0,[s(1),s(end)],'lfoot_heel');
      ks = ks.addKinematicConstraint(kc_lfoot);
      kc_rhand0 = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_pose0,[s(1),s(1)],'rhand0');
      ks = ks.addKinematicConstraint(kc_rhand0);

      r_hand_poseT_relaxed.min=r_hand_poseT-1e-3;
      r_hand_poseT_relaxed.max=r_hand_poseT+1e-3;
      l_hand_poseT_relaxed.min=l_hand_poseT-1e-3;
      l_hand_poseT_relaxed.max=l_hand_poseT+1e-3;
      
         
      kc_rhandT = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_poseT_relaxed,[s(end),s(end)],'rhandT');
      ks = ks.addKinematicConstraint(kc_rhandT);
      kc_lhand0 = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_pose0,[s(1),s(1)],'lhand0');
      ks = ks.addKinematicConstraint(kc_lhand0);
      kc_lhandT = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_poseT_relaxed,[s(end),s(end)],'lhandT');
      ks = ks.addKinematicConstraint(kc_lhandT);
      kc_pelvis = ActionKinematicConstraint(obj.r,pelvis_body,[0;0;0],pelvis_pose0,[s(1),s(end)],'pelvis');
      ks = ks.addKinematicConstraint(kc_pelvis);  
      
if(is_keyframe_constraint)
      % If break point is adjusted via gui.  
     
      r_hand_pose_int_relaxed.min= r_hand_pose_int-1e-3;
      r_hand_pose_int_relaxed.max= r_hand_pose_int+1e-3;
      l_hand_pose_int_relaxed.min= l_hand_pose_int-1e-3;
      l_hand_pose_int_relaxed.max= l_hand_pose_int+1e-3;
      
      if(~isempty(r_ee_goal))
        [~,ind] = min(abs(obj.s_breaks-s_int_r)); % snap to closest break point (avoiding very close double constraints)
        s_int_r=obj.s_breaks(ind);
        kc_rhand_intermediate = ActionKinematicConstraint(obj.r,r_hand_body,[0;0;0],r_hand_pose_int,[s_int_r,s_int_r],'rhand_int');
        ks = ks.addKinematicConstraint(kc_rhand_intermediate);
      end
      if(~isempty(l_ee_goal))
        [~,ind] = min(abs(obj.s_breaks-s_int_l));
        s_int_l=obj.s_breaks(ind);
        kc_lhand_intermediate = ActionKinematicConstraint(obj.r,l_hand_body,[0;0;0],l_hand_pose_int,[s_int_l,s_int_l],'lhand_int');
        ks = ks.addKinematicConstraint(kc_lhand_intermediate);    
      end
    
    
end
 
 % Solve IK at final pose and pass as input to sequence search

 if(~is_keyframe_constraint)      
       rhand_const.min = r_hand_poseT-1e-3;
       rhand_const.max = r_hand_poseT+1e-3;
       lhand_const.min = l_hand_poseT-1e-3;
       lhand_const.max = l_hand_poseT+1e-3;

       
        [q_final_guess,snopt_info] = inverseKin(obj.r,q0,...
              0,comgoal,...
              pelvis_body,[0;0;0],pelvis_pose0,...
              rfoot_body,'heel',r_foot_pose0, ...
              lfoot_body,'heel',l_foot_pose0, ...
              r_hand_body,[0;0;0],rhand_const, ...
              l_hand_body,[0;0;0],lhand_const,...
              ikoptions);

      if(snopt_info == 13)
          warning('The IK fails at the end');
      end
      
      s_breaks=[s(1) s(end)];
      q_breaks=[q0 q_final_guess];
      qtraj_guess = PPTrajectory(foh([s(1) s(end)],[q0 q_final_guess]));
      

 end
 
 % PERFORM IKSEQUENCE OPT
 
      ikseq_options.nSample = obj.num_breaks-1;
      ikseq_options.qdotf.lb = zeros(obj.r.getNumDOF(),1);
      ikseq_options.qdotf.ub = zeros(obj.r.getNumDOF(),1);
      
      if(is_keyframe_constraint)
         ikseq_options.MajorIterationsLimit = 10; 
         ikseq_options.qtraj0 = obj.qtraj_guess_fine; % use previous optimization output as seed
      else
         ikseq_options.MajorIterationsLimit = 100; 
         ikseq_options.qtraj0 = qtraj_guess;
      end      
      
      [s_breaks,q_breaks,qdos_breaks,qddos_breaks,snopt_info] = inverseKinSequence(obj.r,q0,0*q0,ks,ikseq_options);
      if(snopt_info == 13)
          warning('The IK sequence fails');
      end
      xtraj = PPTrajectory(pchipDeriv(s_breaks,[q_breaks;qdos_breaks],[qdos_breaks;qddos_breaks])); 
      xtraj = xtraj.setOutputFrame(obj.r.getStateFrame()); %#ok<*NASGU>
      %%%v = obj.r.constructVisualizer();%v.playback(xtraj,struct('slider',true)); %keyboard;
      %obj.plan_pub.publish(s_breaks,[q_breaks;qdos_breaks]);    
      
      obj.s_breaks = s_breaks;
      obj.q_breaks = q_breaks;
      obj.qdos_breaks = qdos_breaks;
      
      qtraj_guess = PPTrajectory(spline(s_breaks,q_breaks));
      obj.qtraj_guess = qtraj_guess; % cache
      
      % calculate end effectors breaks via FK.
      for brk =1:length(s_breaks),
          kinsol_tmp = doKinematics(obj.r,q_breaks(:,brk));
          rhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,r_hand_body,[0;0;0],2);
          lhand_breaks(:,brk)= forwardKin(obj.r,kinsol_tmp,l_hand_body,[0;0;0],2);
      end
      

      
      q = q_breaks(:,1);
      q_d = q(ind);

      
      
       
      s_total_l =  sum(sqrt(sum(diff(lhand_breaks(1:3,:),1,2).^2,1)));
      s_total_r =  sum(sqrt(sum(diff(rhand_breaks(1:3,:),1,2).^2,1)));
      s_total = max(s_total_l,s_total_r);
      
      res = 0.1; % 10cm res
      s= linspace(0,1,round(s_total/res));
      s = unique([s(:);s_breaks(:)]);

      for i=2:length(s)
          si = s(i);
          tic;
          ikoptions.q_nom = q(:,i-1);
          r_hand_pose_at_t=pose_spline(s_breaks,rhand_breaks,si);  %#ok<*PROP> % evaluate in quaternions
          l_hand_pose_at_t=pose_spline(s_breaks,lhand_breaks,si); % evaluate in quaternions
%           r_hand_pose_at_t=pose_foh(s_breaks,rhand_breaks,si); % evaluate in quaternions
%           l_hand_pose_at_t=pose_foh(s_breaks,lhand_breaks,si); % evaluate in quaternions
      

          if(si~=s(end))
          rhand_const.min = r_hand_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
          rhand_const.max = r_hand_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
          lhand_const.min = l_hand_pose_at_t-1e-2*[ones(3,1);2*ones(4,1)];
          lhand_const.max = l_hand_pose_at_t+1e-2*[ones(3,1);2*ones(4,1)];
          else
          rhand_const.min = r_hand_pose_at_t-1e-4*[ones(3,1);ones(4,1)];
          rhand_const.max = r_hand_pose_at_t+1e-4*[ones(3,1);ones(4,1)];
          lhand_const.min = l_hand_pose_at_t-1e-4*[ones(3,1);ones(4,1)];
          lhand_const.max = l_hand_pose_at_t+1e-4*[ones(3,1);ones(4,1)];
          end
          %l_pose_constraint=[nan;nan;nan;quat2rpy(l_hand_pose_at_t(4:7))];
          %l_pose_constraint=[nan;nan;nan;lhandT(4);lhandT(5);lhandT(6)];
          %l_pose_constraint=[lhandT(1);lhandT(2);lhandT(3);lhandT(4);lhandT(5);lhandT(6)];

          q_guess =qtraj_guess.eval(si);

         [q(:,i),snopt_info] = inverseKin(obj.r,q_guess,...
          0,comgoal,...
          pelvis_body,[0;0;0],pelvis_pose0,...
          rfoot_body,'heel',r_foot_pose0, ...
          lfoot_body,'heel',l_foot_pose0, ...
          r_hand_body,[0;0;0],rhand_const, ...
          l_hand_body,[0;0;0],lhand_const,...
          ikoptions);
               
          
          %
          toc;
          if(snopt_info == 13)
              warning(['The IK fails at ',num2str(s(i))]);
          end
          q_d(:,i) = q(ind,i);
      end
      
      qtraj_guess_fine = PPTrajectory(spline(s, q));
      obj.qtraj_guess_fine = qtraj_guess_fine; % cache   
      
      %   qd_frame = AtlasPositionRef(obj.r);
      %   des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);
      %
      % publish robot plan
      disp('Publishing plan...');
      xtraj = zeros(getNumStates(obj.r)+1,length(s));
      xtraj(1,:) = 0*s;
      for l =1:length(s_breaks),
        ind = find(s == s_breaks(l));
        xtraj(1,ind) = 1.0;
      end
      xtraj(2:getNumDOF(obj.r)+1,:) = q;
      
      
      v_desired = 0.1; % 10cm/sec seconds, hard coded for now
      ts = s.*(s_total/v_desired); % plan timesteps   
      obj.time_2_index_scale = (v_desired/s_total);
      obj.plan_pub.publish(ts,xtraj);
    end
    
    
    function cost = getCostVector(obj)
      cost = Point(obj.r.getStateFrame,1);       
      cost.base_x = 10000;
      cost.base_y = 10000;
      cost.base_z = 10000;
      cost.base_roll = 10000;
      cost.base_pitch = 10000;
      cost.base_yaw = 10000; 
      cost.back_lbz = 10000; 
      cost.back_mby = 10000;
      cost.back_ubx = 10000;
      cost.neck_ay =  10; 
      cost.l_arm_usy = 1;
      cost.l_arm_shx = 1; 
      cost.l_arm_ely = 1; 
      cost.l_arm_elx = 1; 
      cost.l_arm_uwy = 1; 
      cost.l_arm_mwx = 1; 
      cost.l_leg_uhz = 10000; 
      cost.l_leg_mhx = 10000; 
      cost.l_leg_lhy = 10000; 
      cost.l_leg_kny = 10000;
      cost.l_leg_uay = 10000;
      cost.l_leg_lax = 10000;
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
