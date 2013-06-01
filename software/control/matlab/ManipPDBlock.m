classdef ManipPDBlock < MIMODrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
		rhand_body;
		lhand_body;
		cartesian_active_tol % Determines when to turn on the cartesian feedback
		rhand_pts;
		lhand_pts;
		lfoot_body;
		rfoot_body;
		lfoot_pts;
		rfoot_pts;
		num_rhand_pts;
		num_lhand_pts;
		Kp_c; % cartesian feedback proportion gain
    Ki_c;
    lambda; % For L2 penalizing.
    int_interval;
		lc;
		mon;
  end
  
  methods
    function obj = ManipPDBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      coords = AtlasCoordinates(r);
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.nq = getNumDOF(r);

      obj.rhand_body = findLinkInd(r,'r_hand+r_hand_point_mass');
      obj.lhand_body = findLinkInd(r,'l_hand+l_hand_point_mass');
			obj.lfoot_body = findLink(r,'l_foot');
			obj.rfoot_body = findLink(r,'r_foot');

			obj.lfoot_pts = obj.lfoot_body.getContactPoints();
			obj.rfoot_pts = obj.rfoot_body.getContactPoints();
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
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.005;
      end
      
      obj.robot = r;
      
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate

			if(isfield(options,'Kp_c'))
				typecheck(options.Kp_c,'double');
				sizecheck(options.Kp_c,[3,3]);
				if(any(real(eig(options.Kp_c))<-1e-5))
					error('ManipPDBlock: Kp_c must be positive definite');
				end
				obj.Kp_c = options.Kp_c;
			else
				obj.Kp_c = 1*diag([1 1 1]);
      end
      
      if(isfield(options,'Ki_c'))
				typecheck(options.Ki_c,'double');
				sizecheck(options.Ki_c,[3,3]);
				if(any(real(eig(options.Kp_c))<-1e-5))
					error('ManipPDBlock: Ki_c must be positive definite');
				end
				obj.Ki_c = options.Ki_c;
			else
				obj.Ki_c = 0.1*diag([1 1 1]);
      end
			
      obj.int_interval = 1;
      obj.lambda = 1e-6;
      
% 			obj.rhand_pts = [[0;0;0] [0;0.1;0] [0;0;0.1]];
% 			obj.lhand_pts = [[0;0;0] [0;0.1;0] [0;0;0.1]];
      obj.rhand_pts = zeros(3,1);
      obj.lhand_pts = zeros(3,1);
			obj.num_rhand_pts = size(obj.rhand_pts,2);
			obj.num_lhand_pts = size(obj.lhand_pts,2);
			if(isfield(options,'cartesian_active_tol'))
				typecheck(options.cartesian_active_tol,'double');
				sizecheck(options.cartesian_active_tol,[1,1]);
				if(options.cartesian_active_tol<0)
					error('ManipPDBlock: cartesian_active_tol must be positive')
				end
				obj.cartesian_active_tol = options.cartesian_active_tol;
			else
				obj.cartesian_active_tol = 0.015^2*(obj.num_lhand_pts+obj.num_rhand_pts);
			end
			
			obj.lc = lcm.lcm.LCM.getSingleton();
			obj.mon = drake.util.MessageMonitor(drc.robot_plan_t,'utime');
			obj.lc.subscribe('COMMITTED_ROBOT_PLAN',obj.mon);
			controller_data.setField('ee_goal',inf(3,obj.num_rhand_pts+obj.num_lhand_pts));
			controller_data.setField('ee_controller_state',0);
      controller_data.setField('ee_err_int',zeros(3,obj.num_rhand_pts+obj.num_lhand_pts));
      controller_data.setField('int_start_time',0);
      controller_data.setField('int_prev_t',0);
			obj.controller_data = controller_data;
% 				matdata.t = [];
% 				matdata.q = [];
% 				matdata.q_des = [];
% 				matdata.ee_curr = [];
% 				matdata.ee_goal = [];
%         matdata.ee_err = [];
% 				matdata.delta_q = [];
%         matdata.q_err = [];
%         matdata.y = [];
% 				save('cartesian_pd_err.mat','-struct','matdata','t','q','q_des','ee_curr','ee_goal','ee_err','delta_q','q_err','y');
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      q_des = varargin{1};
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
			r = obj.robot;

			ctrl_data = getData(obj.controller_data);
			ee_goal = ctrl_data.ee_goal;
			ee_controller_state = ctrl_data.ee_controller_state;
      ee_err_int = ctrl_data.ee_err_int;
      int_start_time = ctrl_data.int_start_time;
      int_prev_t = ctrl_data.int_prev_t;
			
			data = getNextMessage(obj.mon,0);
			if(~isempty(data))
				display('ManiPDBlock: receive robot plan');
				msg = drc.robot_plan_t(data);
				[xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true);
				q_end = xtraj(1:obj.nq,end);
				kinsol_goal = doKinematics(r,q_end);
				rh_goal = forwardKin(r,kinsol_goal,obj.rhand_body,obj.rhand_pts,0);
				lh_goal = forwardKin(r,kinsol_goal,obj.lhand_body,obj.lhand_pts,0);
				ee_goal = [rh_goal lh_goal];
				ee_controller_state = 1;
			end
			
			% I suppose that the foot position does not change
			if(ee_controller_state ~=0)
				kinsol_curr = doKinematics(r,q);
				[rh_curr,J_rh] = forwardKin(r,kinsol_curr,obj.rhand_body,obj.rhand_pts,0);
				[lh_curr,J_lh] = forwardKin(r,kinsol_curr,obj.lhand_body,obj.lhand_pts,0);
				rf_curr = forwardKin(r,kinsol_curr,obj.rfoot_body,obj.rfoot_pts,1);
				lf_curr = forwardKin(r,kinsol_curr,obj.lfoot_body,obj.lfoot_pts,1);
				ee_curr = [rh_curr lh_curr];
				ee_err = ee_goal-ee_curr;
				ee_err_norm = sum(sum(ee_err.*ee_err,1));
        if(ee_controller_state == 1)
          if(ee_err_norm<obj.cartesian_active_tol)
            int_start_time = t;
            ee_controller_state = 2;
          end
        end
			end

      q_err = [q_des(1:3)-q(1:3);angleDiff(q(4:end),q_des(4:end))];
			if(ee_controller_state == 2||ee_controller_state == 3)
        display(sprintf('The right hand error is %10.6f, %10.6f, %10.6f\n',ee_err(1,1),ee_err(2,1),ee_err(3,1)));        
        J = [J_rh;J_lh];
        J_joint = J(:,7:end);
        ee_err_bnd = 0.01;
        ee_err = max(min(ee_err,ee_err_bnd),-ee_err_bnd);
        delta_q = zeros(obj.nq,1);
        if(ee_controller_state == 2)
          if(t-int_start_time<obj.int_interval)
            ee_err_int = ee_err_int+ee_err*(t-int_prev_t);
            delta_q(7:end) = J_joint'*(J_joint*J_joint'+obj.lambda*eye(3*(obj.num_lhand_pts+obj.num_rhand_pts)))*reshape(obj.Ki_c*ee_err_int,[],1);
            delta_q(34) = 0;
            int_prev_t = t;
          else
            ee_controller_state = 3;
            display('Start proportional control');
          end
        end
        if(ee_controller_state == 3)
          delta_q(7:end) = J_joint'*(J_joint*J_joint'+obj.lambda*eye(3*(obj.num_lhand_pts+obj.num_rhand_pts)))*reshape(obj.Kp_c*ee_err,[],1);
          delta_q(34) = 0;
        end
        q_err = q_err+delta_q;
      end
      
      
			
			y = obj.Kp*q_err-obj.Kd*qd;
      
			obj.controller_data.setField('ee_goal',ee_goal);
			obj.controller_data.setField('ee_controller_state',ee_controller_state);
      obj.controller_data.setField('int_start_time',int_start_time);
      obj.controller_data.setField('ee_err_int',ee_err_int);
      obj.controller_data.setField('int_prev_t',int_prev_t);
    end
  end
  
end
