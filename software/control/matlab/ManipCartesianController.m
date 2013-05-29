classdef ManipCartesianController < MIMODrakeSystem
  % It takes in the cartesian error in the end effector, and the torque output
  % QP controller, and output the aggregated torque with both QP output and
  % pseudo inverse torque to correct the cartesian error
  properties
    nq;
    nu;
    Kp; % proportional gain
		Ki; % integral gain
		Kd; % derivative gain
		lambda; % L2 penalty term to make the JJ'+lambda*I being nonsingular
    controller_data;
    rhand_body;
    lhand_body;
    rhand_pts;
    lhand_pts
    num_rhand_pts;
    num_lhand_pts;
    rh_ee;
    lh_ee;
		rh_lc;
		lh_lc;
		rh_monitor;
		lh_monitor;
    robot;
		cartesian_precision; % when the cartesian error is below this precision value, the pseudo inverse controller is on
		T_palm_hand_r;
		T_palm_hand_l;
		B_inv;
  end
  
  methods
    
    function obj = ManipCartesianController(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      

      if nargin > 2
        typecheck(options,'struct');
      else
        options = struct();
      end
      
      output_frame = r.getInputFrame;
      input_frame = MultiCoordinateFrame({r.getInputFrame,r.getStateFrame});
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setSampleTime(obj,[.005;0]); % why two numbers here?
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.rhand_body = findLinkInd(r,'r_hand+r_hand_point_mass');
      obj.lhand_body = findLinkInd(r,'l_hand+l_hand_point_mass');
      obj.robot = r;
      obj.nq = r.getNumDOF();
      obj.nu = r.getNumInputs();
      
			[~,~,B] = r.manipulatorDynamics(zeros(obj.nq,1),zeros(obj.nq,1));
			obj.B_inv = inv(B(7:end,:));
      if(isfield(options,'Kp'))
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[3,3]) % use quaternions
        if(any(real(eig(options.Kp))<-1e-4))
          error('ManipCartesianController: Kp must be positive definite');
        end
				obj.Kp = options.Kp;
      else
        obj.Kp = 20*diag([1 10 1]);
      end
			
			if(isfield(options,'Ki'))
				typecheck(options.Ki,'double');
				sizecheck(options.Ki,[3,3]);
				if(any(real(eig(options.Ki))<-1e-4))
					error('ManipCartesianController: Ki must be positive definite');
				end
				obj.Ki = options.Ki;
			else
				obj.Ki = 10*diag([1 10 1]);
			end

			if(isfield(options,'Kd'))
				typecheck(options.Kd,'double');
				sizecheck(options.Kd,[3,3]);
				if(any(real(eig(options.Kd))<-1e-4))
					error('ManipCartesianController: Kd must be positive definite');
				end
				obj.Kd = options.Kd;
			else
				obj.Kd = 1*diag([1 10 1]);
			end

			if(isfield(options,'lambda'))
				typecheck(options.lambda,'double');
				sizecheck(options.lambda,[1,1]);
				if(options.lambda<=0)
					error('ManipCartesianController: lambda must be positive');
				end
				obj.lambda = options.lambda;
			else
				obj.lambda = 1e-6;
			end

			if(isfield(options,'cartesian_precision'))
				typecheck(options.cartesian_precision,'double');
				sizecheck(options.cartesian_precision,[1,1]);
				if(options.cartesian_precision<0)
					error('ManipCartesianController: cartesian_precision must be positive');
				end
				obj.cartesian_precision = options.cartesian_precision;
			else
				obj.cartesian_precision = 0.2;
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
      %obj.rh_ee = EndEffector(r,'atlas','right_palm',obj.rhand_pts,'RIGHT_PALM_GOAL');
			%obj.rh_ee.frame.subscribe('RIGHT_PALM_GOAL');
			
			obj.rh_lc = lcm.lcm.LCM.getSingleton();
			obj.rh_monitor = drake.util.MessageMonitor(drc.ee_goal_t,'utime');
			obj.rh_lc.subscribe('RIGHT_PALM_GOAL',obj.rh_monitor);
			obj.lh_lc = lcm.lcm.LCM.getSingleton();
			obj.lh_monitor = drake.util.MessageMonitor(drc.ee_goal_t,'utime');
			obj.lh_lc.subscribe('LEFT_PALM_GOAL',obj.lh_monitor);

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
					matdata.u_qp = [];
					matdata.u_cartesian = [];
					save('cartesian_err.mat','-struct','matdata','t','rh_curr','rh_goal','rh_err','rh_err_int','u_qp','u_cartesian');
    end

    function y = mimoOutput(obj,t,~,varargin)
				u_qp = varargin{1};
				x = varargin{2};
				ctrl_data = getData(obj.controller_data);
				
				typecheck(ctrl_data.rh_goal,'double');
				sizecheck(ctrl_data.rh_goal,[3,obj.num_rhand_pts]);
				typecheck(ctrl_data.lh_goal,'double');
				sizecheck(ctrl_data.lh_goal,[3,obj.num_lhand_pts]);
				rh_goal_pos = ctrl_data.rh_goal(1:3,:);
				rh_err_int = ctrl_data.rh_err_int;
				rh_prev_t = ctrl_data.rh_prev_t;
				%rh_goal_quat = ctrl_data.rh_goal(4:7,:);
				lh_goal_pos = ctrl_data.lh_goal(1:3,:);
				%lh_goal_quat = ctrl_data.lh_goal(4:7,:);
				r = obj.robot;
				q = x(1:obj.nq);
				qd = x(obj.nq+1:end);
				
			  rpalm_goal_data = getNextMessage(obj.rh_monitor,0);
				if(~isempty(rpalm_goal_data))
					display('ManipCartesianController: receive right hand goal');
					rpalm_goal_message = drc.ee_goal_t(rpalm_goal_data);
					rpalm_goal_pos = [rpalm_goal_message.ee_goal_pos.translation.x;rpalm_goal_message.ee_goal_pos.translation.y;rpalm_goal_message.ee_goal_pos.translation.z];
					rpalm_goal_quat = [rpalm_goal_message.ee_goal_pos.rotation.w;rpalm_goal_message.ee_goal_pos.rotation.x;rpalm_goal_message.ee_goal_pos.rotation.y;rpalm_goal_message.ee_goal_pos.rotation.z];
					T_world_palm_r = [quat2rotmat(rpalm_goal_quat) rpalm_goal_pos;0 0 0 1];
					T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
					rh_goal_pos = T_world_hand_r*[obj.rhand_pts;ones(1,obj.num_rhand_pts)];
					rh_goal_pos = rh_goal_pos(1:3,:);
					obj.controller_data.setField('rh_goal',rh_goal_pos);
					obj.controller_data.setField('rh_prev_t',t);
					obj.controller_data.setField('rh_err_int',zeros(3,obj.num_rhand_pts));
					rh_prev_t = t;
					rh_err_int = zeros(3,obj.num_rhand_pts);
				end
				lpalm_goal_data = getNextMessage(obj.lh_monitor,0);
				if(~isempty(lpalm_goal_data))
					display('ManipCartesianController: receive left hand goal');
					lpalm_goal_message = drc.ee_goal_t(lpalm_goal_data);
					lpalm_goal_pos = [lpalm_goal_message.ee_goal_pos.translation.x;lpalm_goal_message.ee_goal_pos.translation.y;lpalm_goal_message.ee_goal_pos.translation.z];
					lpalm_goal_quat = [lpalm_goal_message.ee_goal_pos.rotation.w;lpalm_goal_message.ee_goal_pos.rotation.x;lpalm_goal_message.ee_goal_pos.rotation.y;lpalm_goal_message.ee_goal_pos.rotation.z];
					T_world_palm_l = [quat2rotmat(lpalm_goal_quat) lpalm_goal_pos;0 0 0 1];
					T_world_hand_l = T_world_palm_r*obj.T_palm_hand_r;
					lh_goal_pos = T_world_hand_l*[obj.lhand_pts;ones(1,obj.num_lhand_pts)];
					lh_goal_pos = lh_goal_pos(1:3,:);
					obj.controller_data.setField('lh_goal',lh_goal_pos);
					obj.controller_data.setField('lh_prev_t',t);
					obj.controller_data.setField('lh_err_int',zeros(3,obj.num_lhand_pts));
					lh_prev_t = t;
					lh_err_int = zeros(3,obj.num_lhand_pts);
				end

				kinsol = doKinematics(r,q);
				[rh_curr,J_rh] = forwardKin(r,kinsol,obj.rhand_body,obj.rhand_pts,0);
				[lh_curr,J_lh] = forwardKin(r,kinsol,obj.lhand_body,obj.lhand_pts,0);
				
				lh_err = zeros(3,obj.num_lhand_pts);
				rh_err = zeros(3,obj.num_rhand_pts);
				lh_err(1:3,:) = lh_goal_pos-lh_curr(1:3,:);
				rh_err(1:3,:) = rh_goal_pos-rh_curr(1:3,:);
				%rh_curr_quat = rh_curr(4:7,1);
				%lh_curr_quat = lh_curr(4:7,1);
				%rh_quat_mask = sum(rh_curr_quat.*rh_goal_quat,1)>0;
				%lh_quat_mask = sum(lh_curr_quat.*lh_goal_quat,1)>0;
				%rh_quat_err = rh_goal_quat-rh_curr_quat*rh_quat_mask;
				%lh_quat_err = lh_goal_quat-lh_curr_quat*lh_quat_mask;
				%lh_err(4:7,:) = bsxfun(@times,ones(1,obj.num_lhand_pts),lh_quat_err);
				%rh_err(4:7,:) = bsxfun(@times,ones(1,obj.num_rhand_pts),rh_quat_err);
			

			  lh_pos_err_norm = sum(sqrt(sum(lh_err(1:3,:).*lh_err(1:3,:))));
				%lh_quat_err_norm = acos(abs(lh_err(4:7,1)'*lh_err(4:7,1)));
				%lh_err_norm = lh_pos_err_norm+lh_quat_err_norm; 
			  rh_pos_err_norm = sqrt(sum(rh_err(1:3,:).*rh_err(1:3,:)));
				rh_err_norm = sum(rh_pos_err_norm);
				%rh_quat_err_norm = sum(acos(abs(sum(rh_curr_quat.*rh_goal_quat,1))));
				%rh_err_norm = rh_pos_err_norm+rh_quat_err_norm; 
				if(lh_pos_err_norm<obj.cartesian_precision&&rh_err_norm<obj.cartesian_precision)
					display(sprintf('Cartesian feedback is on for both left and right hands at time %7.2f',t));
					J = [J_rh;J_lh];
					ee_err = [reshape(obj.Kp*rh_err,[],1);reshape(obj.Kp*lh_err,[],1)];
					u_cartesian = (J'/(J*J'+obj.lambda*eye(7*(obj.num_lhand_pts+obj.num_rhand_pts))))*ee_err;
					u_cartesian = u_cartesian(7:end);
				elseif(lh_pos_err_norm<obj.cartesian_precision)
					display(sprintf('Cartesian feedback is on for left hand at time %6.2fs',t));
					J = J_lh;
					ee_err = reshape(obj.Kp*lh_err,[],1);
					u_cartesian = (J'/(J*J'+obj.lambda*eye(7*obj.num_lhand_pts) ))*ee_err;
					u_cartesian = u_cartesian(7:end);
				elseif(rh_err_norm<obj.cartesian_precision)
					display(sprintf('Cartesian feedback is on for right hand at time %6.2fs',t));
					rh_err_int = rh_err_int+rh_err*(t-rh_prev_t);
					J = J_rh;
					ee_err = reshape(obj.Kp*rh_err+obj.Ki*rh_err_int-obj.Kd*reshape(J_rh*qd,3,obj.num_rhand_pts),[],1);
					%u_cartesian = (J'/(J*J'+obj.lambda*ones(7*obj.num_rhand_pts) ))*ee_err;
					%u_cartesian = (J'/(J*J'+obj.lambda*ones(3*obj.num_rhand_pts) ))*ee_err;
					u_cartesian = J'*ee_err;
					u_cartesian = obj.B_inv*u_cartesian(7:end);
					display(sprintf('QP control is                   %10.4f',norm(u_qp)));
					display(sprintf('Additional cartesian control is %10.4f',norm(u_cartesian)));
					obj.controller_data.setField('rh_prev_t',t);
					obj.controller_data.setField('rh_err_int',rh_err_int);
					matdata = load('cartesian_err.mat');
					matdata.t = [matdata.t t];
					matdata.rh_curr = [matdata.rh_err rh_curr(:)];
					matdata.rh_goal = [matdata.rh_goal rh_goal_pos(:)];
					matdata.rh_err = [matdata.rh_err rh_err(:)];
					matdata.rh_err_int = [matdata.rh_err_int rh_err_int(:)];
					matdata.u_qp = [matdata.u_qp u_qp];
					matdata.u_cartesian = [matdata.u_cartesian u_cartesian];
					save('cartesian_err.mat','-struct','matdata','t','rh_curr','rh_goal','rh_err','rh_err_int','u_qp','u_cartesian');
				else
					u_cartesian = zeros(obj.nu,1);
				end

				y = u_qp+u_cartesian;
    end
  end
end
