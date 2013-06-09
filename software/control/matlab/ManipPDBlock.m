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
%     stiff_mon;
    pause_mon;
    joint_names;
    r_arm_ind;
    l_arm_ind;
    rhand_ind;
    lhand_ind;
    rhand_pts;
    lhand_pts;
    private_data;
    lambda;
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
        
      
      obj.Kp_stiff = eye(6);
      obj.Kp_comp = diag([0.95 0.7 0.95 0.7 0.7 0.7]);
      
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
      obj.rhand_pts = [0;0;0];
      obj.lhand_pts = [0;0;0];
      obj.lambda = 1e-6;
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.private_data = SharedDataHandle(struct(...
        'rhand_force_cached',[],...
        'lhand_force_cached',[],...
        'rhand_contact_time_cached',[],...
        'lhand_contact_time_cached',[]));

      lc = lcm.lcm.LCM.getSingleton();
      obj.pause_mon = drake.util.MessageMonitor(drc.plan_control_t,'utime');
      lc.subscribe('COMMITTED_PLAN_PAUSE',obj.pause_mon);
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      try
      q_des = varargin{1};
      x = varargin{2};
      hand_ft = varargin{3};
      rhand_force = hand_ft(7:9);
      lhand_force = hand_ft(1:3);
      rhand_torque = hand_ft(10:12);
      rhand_force_norm = norm(rhand_force);
      lhand_force_norm = norm(lhand_force);
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      r = obj.robot;
      
      pause_data = getNextMessage(obj.pause_mon,0);
      if(~isempty(pause_data))
        pause_msg = drc.COMMITTED_PLAN_PAUSE(pause_data);
        if(pause_msg.control == drc.COMMITTED_PLAN_PAUSE.PAUSE)
          obj.controller_data.setField('qtraj',q_des);
        end
      end
      pri_data = getData(obj.private_data);
      rhand_force_cached = pri_data.rhand_force_cached;
      lhand_force_cached = pri_data.lhand_force_cached;
      rhand_contact_time_cached = pri_data.rhand_contact_time_cached;
      lhand_contact_time_cached = pri_data.lhand_contact_time_cached;
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
      if(~isempty(rhand_force_cached))
        
%         if(rhand_force_norm-rhand_force_norm_cached>70)
%           display('Detect contact on right hand');
%           rhand_contact_time_cached = t;
%         end
      end
      if(l_arm_control_type ~= drc.robot_plan_t.NONE)
        free_joint_ind = free_joint_ind(~ismember(free_joint_ind,obj.l_arm_ind));
        constraint_joint_ind = [constraint_joint_ind obj.l_arm_ind];
      end
      if(r_arm_control_type ~= drc.robot_plan_t.NONE)
        free_joint_ind = free_joint_ind(~ismember(free_joint_ind,obj.r_arm_ind));
        constraint_joint_ind = [constraint_joint_ind obj.r_arm_ind];
      end
      
      
      err_q = [q_des(1:3)-q(1:3);angleDiff(q(4:end),q_des(4:end))];
      y = obj.Kp*err_q - obj.Kd*qd;
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
%             display(sprintf('%10.5f, %10.5f, %10.5f,%10.5f, %10.5f, %10.5f',...
%               rhand_force(1),rhand_force(2),rhand_force(3),rhand_torque(1),rhand_torque(2),rhand_torque(3)));
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
%           kinsol_des = doKinematics(r,q_des);
%           rh_des = forwardKin(r,kinsol_des,obj.rhand_ind, obj.rhand_pts,1);
%           ee_goal = rh_des;
%           kinsol_curr = doKinematics(r,q);
%           [rh_curr,J_rh] = forwardKin(r,kinsol_curr,obj.rhand_ind,obj.rhand_pts,1);
%           ee_curr = rh_curr;
%           ee_err = ee_goal-ee_curr;
%           ee_err(4:6,:) = angleDiff(ee_curr(4:6,:),ee_goal(4:6,:));
%           J_rarm = J_rh(:,obj.r_arm_ind);
%           J_free = J_rh(:,free_joint_ind);
% %           q_err_free = err_q(free_joint_ind);
%           q_err_rarm = (J_rarm'/(J_rarm*J_rarm'+obj.lambda*eye(length(obj.r_arm_ind))))*Kp_r_arm*(ee_err-J_free*q_err_free);
% %           q_err_rarm = (J_rarm'/(J_rarm*J_rarm'+obj.lambda*eye(length(obj.r_arm_ind))))*Kp_r_arm*(ee_err);
% %           q_err_rarm = J_rarm'*Kp_r_arm*(ee_err);
%           display(sprintf('The cartesian error is %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f',ee_err(1),ee_err(2),ee_err(3),ee_err(4),ee_err(5),ee_err(6)));
% %           display(sprintf('The original arm joint error is  %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f',...
% %             err_q(obj.r_arm_ind(1)),err_q(obj.r_arm_ind(2)),err_q(obj.r_arm_ind(3)),err_q(obj.r_arm_ind(4)),err_q(obj.r_arm_ind(5)),err_q(obj.r_arm_ind(6))));
%           err_q(obj.r_arm_ind) = q_err_rarm;
% %           display(sprintf('The compliant arm joint error is %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f',...
% %             err_q(obj.r_arm_ind(1)),err_q(obj.r_arm_ind(2)),err_q(obj.r_arm_ind(3)),err_q(obj.r_arm_ind(4)),err_q(obj.r_arm_ind(5)),err_q(obj.r_arm_ind(6))));
%           y = obj.Kp*err_q-obj.Kd*qd;
          
        
      end
      obj.private_data.setField('rhand_force_cached',rhand_force);
      obj.private_data.setField('lhand_force_cached',lhand_force);
      obj.private_data.setField('rhand_contact_time_cached',rhand_contact_time_cached);
      obj.private_data.setField('lhand_contact_time_cached',lhand_contact_time_cached);
      catch err
        keyboard;
      end
    end
  end
  
end
