classdef ManipPDBlock < MIMODrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  % For hand end effectors, choose Kp_hand and Kd_hand to adjust the stiffness.
  properties
    nq;
    Kp;
    Kd;
    Kp_stiff;
    Kp_comp;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    plan_mon;
    pause_mon;
    joint_names;
    r_arm_ind;
    l_arm_ind;
    rhand_ind;
    lhand_ind;
    head_ind;
    rfoot_ind;
    lfoot_ind;
    rhand_pts;
    lhand_pts;
    rpalm_pts;
    lpalm_pts;
    head_pts;
    rfoot_pts;
    lfoot_pts;
    private_data;
    lambda;
    ee_teleop_mon;
%     contact_detect; % This should only be used in mating task!!!!!!
  end
  
  methods
    function obj = ManipPDBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      coords = AtlasCoordinates(r);
      hand_ft_frame = AtlasHandForceTorque();
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame,hand_ft_frame});
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);
      
      
      if nargin<3
        options = struct();
      end
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[obj.nq obj.nq]);
        obj.Kp = options.Kp;
      else
        obj.Kp = 170.0*eye(obj.nq);
%        obj.Kp([1,2,6],[1,2,6]) = zeros(3); % ignore x,y,yaw
      end        
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[obj.nq obj.nq]);
        obj.Kd = options.Kd;
      else
        obj.Kd = 19.0*eye(obj.nq);
 %       obj.Kd([1,2,6],[1,2,6]) = zeros(3); % ignore x,y,yaw
      end
        
  	  if isfield(options,'soft_ankles')
        typecheck(options.soft_ankles,'logical');
        if options.soft_ankles
          state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
          lax_idx = find(~cellfun(@isempty,strfind(state_names,'lax')));
          uay_idx = find(~cellfun(@isempty,strfind(state_names,'uay')));
          obj.Kp(uay_idx,uay_idx) = 5*eye(2);
          obj.Kp(lax_idx,lax_idx) = 5*eye(2);
          obj.Kd(uay_idx,uay_idx) = 0.01*eye(2);
          obj.Kd(lax_idx,lax_idx) = 0.01*eye(2);
        end
      end    
      obj.Kp_stiff = eye(6);
      obj.Kp_comp = diag([0.95 0.75 0.95 0.75 0.75 0.75]);
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.005;
      end
      
      obj.robot = r;
      obj.joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
%       obj.r_arm_ind = find(~cellfun(@isempty,strfind(obj.joint_names,'r_arm'))&...
%         cellfun(@isempty,strfind(obj.joint_names,'usy'))&...
%         cellfun(@isempty,strfind(obj.joint_names,'shx')));
%       obj.l_arm_ind = find(~cellfun(@isempty,strfind(obj.joint_names,'l_arm'))&...
%         cellfun(@isempty,strfind(obj.joint_names,'usy'))&...
%         cellfun(@isempty,strfind(obj.joint_names,'shx')));
      obj.r_arm_ind = find(~cellfun(@isempty,strfind(obj.joint_names,'r_arm')));
      obj.l_arm_ind = find(~cellfun(@isempty,strfind(obj.joint_names,'l_arm')));
      
      obj.rhand_ind = obj.robot.findLinkInd('r_hand+r_hand_point_mass');
      obj.lhand_ind = obj.robot.findLinkInd('l_hand+l_hand_point_mass');
      obj.rfoot_ind = obj.robot.findLinkInd('r_foot');
      obj.lfoot_ind = obj.robot.findLinkInd('l_foot');
      obj.head_ind = obj.robot.findLinkInd('head');
      % NOTE: with fixed joint hands, the link name is huge. hence using r.findLinkInd('r_foot')+1
      % fixedjoint hands does not work, reverting to point hands
      % obj.rhand_ind = r.findLinkInd('r_foot')+1; %obj.robot.findLinkInd('r_hand+r_hand_point_mass');
      % obj.lhand_ind = r.findLinkInd('l_foot')+1; %obj.robot.findLinkInd('l_hand+l_hand_point_mass');
      
      obj.rhand_pts = [0;0;0];
      obj.lhand_pts = [0;0;0];
      obj.rpalm_pts = [0;-0.1;0];
      obj.lpalm_pts = [0;0.1;0];
      obj.rfoot_pts = obj.robot.findLink('r_foot').getContactPoints();
      obj.lfoot_pts = obj.robot.findLink('l_foot').getContactPoints();
      obj.head_pts = [0;0;0];
      
      obj.lambda = 1e-6;
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.private_data = SharedDataHandle(struct(...
        'lpalm_goal',[],...
        'rpalm_goal',[],...
        'lf_goal',[],...
        'rf_goal',[],...
        'h_goal',[],...
        'com_const',[],...
        'plan_state',0)); % plan_state = 0 if listening to COMMITTED_ROBOT_PLAN, plan_state = 1 if using approximateIK to solve the teleop problem iteratively. plan = 2 if teleop problem is solved by IK already.
                        

      lc = lcm.lcm.LCM.getSingleton();
      obj.pause_mon = drake.util.MessageMonitor(drc.plan_control_t,'utime');
      lc.subscribe('COMMITTED_PLAN_PAUSE',obj.pause_mon);
      
      lc = lcm.lcm.LCM.getSingleton();
      obj.ee_teleop_mon = drake.util.MessageMonitor(drc.ee_cartesian_adjust_t,'utime');
      lc.subscribe('COMMITTED_EE_ADJUSTMENT',obj.ee_teleop_mon);
      lc = lcm.lcm.LCM.getSingleton();
      obj.plan_mon = drake.util.MessageMonitor(drc.robot_plan_t,'utime');
      lc.subscribe('COMMITTED_ROBOT_PLAN',obj.plan_mon);
      
%       obj.contact_detect = mateContactDetectMachine();
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      try
      q_des = varargin{1};
      x = varargin{2};
      hand_ft = varargin{3};
%       rhand_force = hand_ft(7:9);
%       lhand_force = hand_ft(1:3);
%       rhand_torque = hand_ft(10:12);
%       rhand_force_norm = norm(rhand_force);
%       lhand_force_norm = norm(lhand_force);
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      r = obj.robot;
      
      
%       kinsol = doKinematics(r,q);
%       rhand_force_world = forwardKin(r,kinsol,obj.rhand_ind,rhand_force,1);
%       rhand_torque_world = forwardKin(r,kinsol,obj.rhand_ind,rhand_torque,1);
%       display(sprintf('%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f',...
%         rhand_force(1),rhand_force(2),rhand_force(3),rhand_torque(1),rhand_torque(2),rhand_torque(3),...
%         rhand_force_world(1),rhand_force_world(2),rhand_force_world(3),rhand_torque_world(1),rhand_torque_world(2),rhand_torque_world(3)));
      pause_data = getNextMessage(obj.pause_mon,0);
      if(~isempty(pause_data))
        pause_msg = drc.plan_control_t(pause_data);
        if(pause_msg.control == drc.plan_control_t.PAUSE)
          obj.controller_data.setField('qtraj',q_des);
        end
      end
      
%       ee_teleop_data = getNextMessage(obj.ee_teleop_mon,0);
%       if(~isempty(ee_teleop_data))
%         display('ManipPDBlock: receive ee teleop command');
%         ee_teleop_msg = drc.ee_cartesian_adjust_t(ee_teleop_data);
%         ee_delta_pos = [ee_teleop_msg.pos_delta.x;ee_teleop_msg.pos_delta.z;ee_teleop_msg.pos_delta.y];
%         ee_delta_rpy = [ee_teleop_msg.rpy_delta.x;ee_teleop_msg.rpy_delta.z;ee_teleop_msg.rpy_delta.y];
%         kinsol_curr = doKinematics(r,q);
%         rpalm_curr = forwardKin(r,kinsol_curr,obj.rhand_ind,obj.rpalm_pts,1);
%         lpalm_curr = forwardKin(r,kinsol_curr,obj.lhand_ind,obj.lpalm_pts,1);
%         rpalm_goal = rpalm_curr;
%         lpalm_goal = lpalm_curr;
%         if(ee_teleop_msg.RIGHT_HAND)
%           rpalm_goal(1:3) = rpalm_goal(1:3)+ee_delta_pos;
%           rpalm_goal(4:6) = rotmat2rpy(rpy2rotmat(ee_delta_rpy)*rpy2rotmat(rpalm_goal(4:6)));
%         elseif(ee_teleop_msg.LEFT_HAND)
%           lpalm_goal = lpalm_goal(1:3)+ee_delta_pos;
%           lpalm_goal(4:6) = rotmat2rpy(rpy2rotmat(ee_delta_rpy)*rpy2rotmat(lpalm_goal(4:6)));
%         end
%         head_curr = forwardKin(r,kinsol_curr,obj.head_ind,obj.head_pts,1);
%         rf_curr = forwardKin(r,kinsol_curr,obj.rfoot_ind,obj.rfoot_pts,0);
%         lf_curr = forwardKin(r,kinsol_curr,obj.lfoot_ind,obj.lfoot_pts,0);
%         com_curr = getCOM(r,kinsol_curr);
%         com_const = struct('max',com_curr+1e-3*ones(3,1),'min',com_curr-1e-3*ones(3,1));
%         obj.controller_data.setField('qtraj',q_des);
%         obj.private_data.setField('plan_state',1);
%         obj.private_data.setField('lpalm_goal',lpalm_goal);
%         obj.private_data.setField('rpalm_goal',rpalm_goal);
%         obj.private_data.setField('h_goal',head_curr);
%         obj.private_data.setField('rf_goal',rf_curr);
%         obj.private_data.setField('lf_goal',lf_curr);
%         obj.private_data.setField('com_const',com_const);
%       end
%       plan_data = getNextMessage(obj.plan_mon,0);
%       if(~isempty(plan_data))
%         obj.private_data.setField('plan_state',0);
%       end
%       
%       pri_data = getData(obj.private_data);
%       plan_state = pri_data.plan_state;
%       if(plan_state == 1)
%         rpalm_goal = pri_data.rpalm_goal;
%         lpalm_goal = pri_data.lpalm_goal;
%         lf_goal = pri_data.lf_goal;
%         rf_goal = pri_data.rf_goal;
%         h_goal = pri_data.h_goal;
%         com_const = pri_data.com_const;
%         ikargs = {obj.rhand_ind,obj.rpalm_pts,rpalm_goal,...
%           obj.lhand_ind,obj.lpalm_pts,lpalm_goal,obj.head_ind,obj.head_pts,h_goal,...
%           obj.rfoot_ind,obj.rfoot_pts,rf_goal,obj.lfoot_ind,obj.lfoot_pts,lf_goal,...
%           0,com_const};
%         [q_des,info] = approximateIK(r,q,ikargs{:},struct('q_nom',q));
%         if info
%           fprintf(1,'warning: approximate IK failed.  calling IK\n');
%           [q_des,info] = inverseKin(r,q,ikargs{:},struct('q_nom',q));
%           if(info <10)
%             obj.controller_data.setField('qtraj',q_des);
%             obj.private_data.setField('plan_state',2);
%           end
%         end
%       end




%       contact_flag = obj.contact_detect.update(rhand_torque(1));
%       if(contact_flag)
%         display(sprintf('Detect contact at time %7.4f',t));
%         obj.contact_detect.restart_machine();
%       end
%       pri_data = getData(obj.private_data);
%       rhand_force_cached = pri_data.rhand_force_cached;
%       lhand_force_cached = pri_data.lhand_force_cached;
%       rhand_contact_time_cached = pri_data.rhand_contact_time_cached;
%       lhand_contact_time_cached = pri_data.lhand_contact_time_cached;
%       control_state = pri_data.control_state;
%       ee_goal = pri_data.ee_goal;
%       arms_control_type = pri_data.arms_control_type;
%       Kp_r_arm = pri_data.Kp_r_arm;
%       Kp_l_arm = pri_data.Kp_l_arm;
      
      ctrl_data = getData(obj.controller_data);
      l_arm_control_type = ctrl_data.l_arm_control_type;
      r_arm_control_type = ctrl_data.r_arm_control_type;
      free_joint_ind = 1:obj.nq;
      constraint_joint_ind = [];
%       if(~isempty(rhand_force_cached))
        
%         if(rhand_force_norm-rhand_force_norm_cached>70)
%           display('Detect contact on right hand');
%           rhand_contact_time_cached = t;
%         end
%       end
      if(l_arm_control_type ~= drc.robot_plan_t.NONE)
        free_joint_ind = free_joint_ind(~ismember(free_joint_ind,obj.l_arm_ind));
        constraint_joint_ind = [constraint_joint_ind obj.l_arm_ind];
      end
      if(r_arm_control_type ~= drc.robot_plan_t.NONE)
        free_joint_ind = free_joint_ind(~ismember(free_joint_ind,obj.r_arm_ind));
        constraint_joint_ind = [constraint_joint_ind obj.r_arm_ind];
      end
      
      
      err_q = [q_des(1:3)-q(1:3);angleDiff(q(4:end),q_des(4:end))];
      y = max(-100*ones(obj.nq,1),min(100*ones(obj.nq,1),obj.Kp*err_q - obj.Kd*qd));
      if(l_arm_control_type ~= drc.robot_plan_t.NONE||...
          r_arm_control_type ~= drc.robot_plan_t.NONE)
        kinsol_des = doKinematics(r,q_des);
        lh_des = forwardKin(r,kinsol_des,obj.lhand_ind,obj.lhand_pts,1);
        rh_des = forwardKin(r,kinsol_des,obj.rhand_ind,obj.rhand_pts,1);
        kinsol_curr = doKinematics(r,q);
        ee_goal = [];
        ee_curr = [];
        J = [];
        Kp_r_arm = [];
        Kp_l_arm = [];
        if(l_arm_control_type ~= drc.robot_plan_t.NONE)
          [lh_curr,J_lh] = forwardKin(r,kinsol_curr,obj.lhand_ind,obj.lhand_pts,1);
          J = [J;J_lh];
          ee_goal = [ee_goal lh_des];
          ee_curr = [ee_curr lh_curr];
          if l_arm_control_type == drc.robot_plan_t.POSITION
          elseif l_arm_control_type == drc.robot_plan_t.IMPEDANCE
          elseif l_arm_control_type == drc.robot_plan_t.STIFF
            Kp_l_arm = eye(6);
          elseif l_arm_control_type == drc.robot_plan_t.COMPLIANT
            Kp_l_arm = obj.Kp_comp;
          end
        end
        if(r_arm_control_type ~= drc.robot_plan_t.NONE)
          [rh_curr,J_rh] = forwardKin(r,kinsol_curr,obj.rhand_ind,obj.rhand_pts,1);
          J = [J;J_rh];
          ee_goal = [ee_goal rh_des];
          ee_curr = [ee_curr rh_curr];
          if r_arm_control_type == drc.robot_plan_t.POSITION
          elseif r_arm_control_type == drc.robot_plan_t.IMPEDANCE
          elseif r_arm_control_type == drc.robot_plan_t.STIFF
            Kp_r_arm = eye(6);
          elseif r_arm_control_type == drc.robot_plan_t.COMPLIANT
            Kp_r_arm = obj.Kp_comp;
          end
        end
        ee_err = ee_goal-ee_curr;
        ee_err(4:6,:) = angleDiff(ee_curr(4:6,:),ee_goal(4:6,:));
%         display(sprintf('The cartesian error is %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f',ee_err(1),ee_err(2),ee_err(3),ee_err(4),ee_err(5),ee_err(6)));
        q_err_free = err_q(free_joint_ind);
        J_free = J(:,free_joint_ind);
        J_constraint = J(:,constraint_joint_ind);
        J_constraint_inv = J_constraint'/(J_constraint*J_constraint'+obj.lambda*eye(length(constraint_joint_ind)));
        q_err_constraint = J_constraint_inv*(blkdiag(Kp_l_arm,Kp_r_arm)*ee_err-J_free*q_err_free);
        err_q(constraint_joint_ind) = q_err_constraint;
        y = obj.Kp*err_q-obj.Kd*qd;
      end
%       obj.private_data.setField('rhand_force_cached',rhand_force);
%       obj.private_data.setField('lhand_force_cached',lhand_force);
%       obj.private_data.setField('rhand_contact_time_cached',rhand_contact_time_cached);
%       obj.private_data.setField('lhand_contact_time_cached',lhand_contact_time_cached);
      catch err
        keyboard;
      end
    end
    
  end
  
end
