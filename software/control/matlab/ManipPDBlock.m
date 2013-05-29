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
		T_palm_hand_r;
		T_palm_hand_l;
		cartesian_active_tol % Determines when to turn on the cartesian feedback
		rhand_pts;
		lhand_pts;
		num_rhand_pts;
		num_lhand_pts;
		rpalm_monitor;
		lpalm_monitor;
		Kp_c; % cartesian feedback proportion gain
		Ki_c; % cartesian feedback integral gain
  end
  
  methods
    function obj = ManipPDBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      disp('run the controller that should eliminate steady state error for left and right hands');
      coords = AtlasCoordinates(r);
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.nq = getNumDOF(r);

      obj.rhand_body = findLinkInd(r,'r_hand+r_hand_point_mass');
      obj.lhand_body = findLinkInd(r,'l_hand+l_hand_point_mass');

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
				obj.Kp_c = 0.1*diag([1 1 1]);
			end
			
			if(isfield(options,'Ki_c'))
				typecheck(options.Ki_c,'double');
				sizecheck(options.Ki_c,[3,3]);
				if(any(real(eig(options.Ki_c))<-1e-5))
					error('ManipPDBlock: Ki_c must be positive definite');
				end
				obj.Ki_c = options.Ki_c;
			else
				obj.Ki_c = 1*diag([1 1 1]);
			end

			if(isfield(options,'cartesian_active_tol'))
				typecheck(options.cartesian_active_tol,'double');
				sizecheck(options.cartesian_active_tol,[1,1]);
				if(options.cartesian_active_tol<0)
					error('ManipPDBlock: cartesian_active_tol must be positive')
				end
				obj.cartesian_active_tol = options.cartesian_active_tol;
			else
				obj.cartesian_active_tol = 0.2;
			end

			if(isfield(options,'rpalm_offset'))
				typecheck(options.rpalm_offset,'double');
				sizecheck(options.rpalm_offset,[6,1]);
				rpalm_offset = options.rpalm_offset;
			else
				rpalm_offset = [0;-0.1;0;-pi/2;0;-pi/2];
			end
			obj.T_palm_hand_r = [rpy2rotmat(rpalm_offset(4:6))' -rpy2rotmat(rpalm_offset(4:6))'*rpalm_offset(1:3);0 0 0 1];
			if(isfield(options,'lpalm_offset'))
				typecheck(options.lpalm_offset,'double');
				sizecheck(options.lpalm_offset,[6,1]);
				lpalm_offset = options.lpalm_offset;
			else
				lpalm_offset = [0;0.1;0;pi/2;0;pi/2];
			end
			obj.T_palm_hand_l = [rpy2rotmat(lpalm_offset(4:6))' -rpy2rotmat(lpalm_offset(4:6))'*lpalm_offset(1:3);0 0 0 1];
			
      if(isfield(options,'rhand_pts'))
				typecheck(options.rhand_pts,'double');
				sizecheck(options.rhand_pts,[3,nan]);
				obj.rhand_pts = options.rhand_pts;
      else
				obj.rhand_pts = [[0;0;0] [0;-0.1;0] [0;0;-0.1]];
      end
      obj.num_rhand_pts = size(obj.rhand_pts,2);

      if(isfield(options,'lhand_pts'))
				typecheck(options.lhand_pts,'double');
				sizecheck(options.lhand_pts,[3,nan]);
				obj.lhand_pts = options.lhand_pts;
      else
				obj.lhand_pts = [[0;0;0] [0;0.1;0] [0;0;0.1]];
      end
      obj.num_lhand_pts = size(obj.lhand_pts,2);

		  rpalm_lc = lcm.lcm.LCM.getSingleton();
			obj.rpalm_monitor = drake.util.MessageMonitor(drc.ee_goal_t,'utime');
			rpalm_lc.subscribe('RIGHT_PALM_GOAL',obj.rpalm_monitor);

		  lpalm_lc = lcm.lcm.LCM.getSingleton();
			obj.lpalm_monitor = drake.util.MessageMonitor(drc.ee_goal_t,'utime');
			lpalm_lc.subscribe('LEFT_PALM_GOAL',obj.lpalm_monitor);

			controller_data.setField('rh_goal',inf(3,obj.num_rhand_pts));
			controller_data.setField('lh_goal',inf(3,obj.num_lhand_pts));
			controller_data.setField('rh_err_int',zeros(3,obj.num_rhand_pts));
			controller_data.setField('rh_prev_t',0);
			controller_data.setField('lh_err_int',zeros(3,obj.num_lhand_pts));
			controller_data.setField('lh_prev_t',0);

			obj.controller_data = controller_data;
				matdata.t = [];
				matdata.rh_curr = [];
				matdata.rh_goal = [];
				matdata.rh_err = [];
				matdata.rh_err_int = [];
				matdata.qdd_err = [];
				matdata.qdd_cartesian = [];
				save('cartesian_pd_err.mat','-struct','matdata','t','rh_curr','rh_goal','rh_err','rh_err_int','qdd_err','qdd_cartesian');
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      q_des = varargin{1};
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
			r = obj.robot;

			controller_data = getData(obj.controller_data);
			typecheck(controller_data.rh_goal,'double');
			sizecheck(controller_data.rh_goal,[3,obj.num_rhand_pts]);
			typecheck(controller_data.lh_goal,'double');
			sizecheck(controller_data.lh_goal,[3,obj.num_lhand_pts]);
			rh_goal = controller_data.rh_goal;
			lh_goal = controller_data.lh_goal;
			rh_err_int = controller_data.rh_err_int;
			rh_prev_t = controller_data.rh_prev_t;
			lh_err_int = controller_data.lh_err_int;
			lh_prev_t = controller_data.lh_prev_t;
			
			rpalm_goal_data = getNextMessage(obj.rpalm_monitor,0);
			if(~isempty(rpalm_goal_data))
				display('ManipPDBlock: receive right palm goal');
				rpalm_goal_message = drc.ee_goal_t(rpalm_goal_data);
				rpalm_goal_pos = [rpalm_goal_message.ee_goal_pos.translation.x;rpalm_goal_message.ee_goal_pos.translation.y;rpalm_goal_message.ee_goal_pos.translation.z];
				rpalm_goal_quat = [rpalm_goal_message.ee_goal_pos.rotation.w;rpalm_goal_message.ee_goal_pos.rotation.x;rpalm_goal_message.ee_goal_pos.rotation.y;rpalm_goal_message.ee_goal_pos.rotation.z];
				T_world_palm_r = [quat2rotmat(rpalm_goal_quat) rpalm_goal_pos;0 0 0 1];
				T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
				rh_goal = T_world_hand_r*[obj.rhand_pts;ones(1,obj.num_rhand_pts)];
				rh_goal = rh_goal(1:3,:);
				obj.controller_data.setField('rh_goal',rh_goal);
				obj.controller_data.setField('rh_prev_t',t);
				obj.controller_data.setField('rh_err_int',zeros(3,obj.num_rhand_pts));
			end
			lpalm_goal_data = getNextMessage(obj.lpalm_monitor,0);
			if(~isempty(lpalm_goal_data))
				display('ManipPDBlock: receive left palm goal');
				lpalm_goal_message = drc.ee_goal_t(lpalm_goal_data);
				lpalm_goal_pos = [lpalm_goal_message.ee_goal_pos.translation.x;lpalm_goal_message.ee_goal_pos.translation.y;lpalm_goal_message.ee_goal_pos.translation.z];
				lpalm_goal_quat = [lpalm_goal_message.ee_goal_pos.rotation.w;lpalm_goal_message.ee_goal_pos.rotation.x;lpalm_goal_message.ee_goal_pos.rotation.y;lpalm_goal_message.ee_goal_pos.rotation.z];
				T_world_palm_l = [quat2rotmat(lpalm_goal_quat) lpalm_goal_pos;0 0 0 1];
				T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
				lh_goal = T_world_hand_l*[obj.lhand_pts;ones(1,obj.num_lhand_pts)];
				lh_goal = lh_goal(1:3,:);
				obj.controller_data.setField('lh_goal',lh_goal);
				obj.controller_data.setField('lh_prev_t',t);
				obj.controller_data.setField(lh_err_int',zeros(3,obj.num_lhand_pts));
			end

      kinsol_curr = doKinematics(obj.robot,q,false,true);
			[rh_curr,J_rh] = forwardKin(r,kinsol_curr,obj.rhand_body,obj.rhand_pts,0);
			[lh_curr,J_lh] = forwardKin(r,kinsol_curr,obj.lhand_body,obj.lhand_pts,0);

			lh_err = lh_goal-lh_curr;
			rh_err = rh_goal-rh_curr;
			
			rh_err_norm = sum(sqrt(sum(rh_err.*rh_err,1)));
			lh_err_norm = sum(sqrt(sum(lh_err.*lh_err,1)));

      err_q = q_des - q;
      qdd_err = obj.Kp*err_q - obj.Kd*qd;
			if(lh_err_norm<obj.cartesian_active_tol&&rh_err_norm<obj.cartesian_active_tol)
			elseif(lh_err_norm<obj.cartesian_active_tol)
			elseif(rh_err_norm<obj.cartesian_active_tol)
				display(sprintf('Cartesian manipulation block is on for right hand at time %7.3fs',t));
				rh_err_int = rh_err_int+rh_err*(t-rh_prev_t);
				J = J_rh;
				ee_err = reshape(obj.Kp_c*rh_err+obj.Ki_c*rh_err_int,[],1);
				qdd_cartesian = J'*ee_err;
        obj.controller_data.setField('rh_err_int',rh_err_int);
        obj.controller_data.setField('rh_prev_t',t);
					matdata = load('cartesian_pd_err.mat');
					matdata.t = [matdata.t t];
					matdata.rh_curr = [matdata.rh_err rh_curr(:)];
					matdata.rh_goal = [matdata.rh_goal rh_goal(:)];
					matdata.rh_err = [matdata.rh_err rh_err(:)];
					matdata.rh_err_int = [matdata.rh_err_int rh_err_int(:)];
					matdata.qdd_err = [matdata.qdd_err,qdd_err];
					matdata.qdd_cartesian = [matdata.qdd_cartesian qdd_cartesian];
					save('cartesian_pd_err.mat','-struct','matdata','t','rh_curr','rh_goal','rh_err','rh_err_int','qdd_err','qdd_cartesian');
			else
				qdd_cartesian = zeros(obj.nq,1);
			end
			y = qdd_err+qdd_cartesian;
    end
  end
  
end
