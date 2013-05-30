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
		mon;
		lc;
    robot;
		cartesian_active_tol; % when the cartesian error is below this precision value, the pseudo inverse controller is on
		T_palm_hand_r;
		T_palm_hand_l;
		B_inv;
		integral_bnd;
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
        obj.Kp = 500;
      end
			
			if(isfield(options,'Ki'))
				typecheck(options.Ki,'double');
				sizecheck(options.Ki,[3,3]);
				if(any(real(eig(options.Ki))<-1e-4))
					error('ManipCartesianController: Ki must be positive definite');
				end
				obj.Ki = options.Ki;
			else
				obj.Ki = 0*diag([1 10 1]);
			end

			if(isfield(options,'Kd'))
				typecheck(options.Kd,'double');
				sizecheck(options.Kd,[3,3]);
				if(any(real(eig(options.Kd))<-1e-4))
					error('ManipCartesianController: Kd must be positive definite');
				end
				obj.Kd = options.Kd;
			else
				obj.Kd = 0*diag([1 1 1]);
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

			if(isfield(options,'cartesian_active_tol'))
				typecheck(options.cartesian_active_tol,'double');
				sizecheck(options.cartesian_active_tol,[1,1]);
				if(options.cartesian_active_tol<0)
					error('ManipCartesianController: cartesian_active_tol must be positive');
				end
				obj.cartesian_active_tol = options.cartesian_active_tol;
			else
				obj.cartesian_active_tol = 0.2;
			end

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

			if(isfield(options,'integral_bnd'))
				typecheck(options.integral_bnd,'double');
				sizecheck(options.integral_bnd,[1,1]);
				if(options.integral_bnd<=0)
					error('ManipCartesianController: integral bound must be positive');
				end
				obj.integral_bnd = options.integral_bnd;
			else
				obj.integral_bnd = 0.2;
			end
			
			obj.lc = lcm.lcm.LCM.getSingleton();
			obj.mon = drake.util.MessageMonitor(drc.robot_plan_t,'utime');
			obj.lc.subscribe('COMMITTED_ROBOT_PLAN',obj.mon);


			controller_data.setField('ee_goal',inf(3,obj.num_rhand_pts+obj.num_lhand_pts));
			controller_data.setField('ee_prev_t',[0,0]);
			controller_data.setField('ee_err_int',zeros(3,obj.num_lhand_pts+obj.num_lhand_pts));
			controller_data.setField('ee_controller_state',0);
			controller_data.setField('u_cartesian',[]);
			controller_data.setField('u_qp',[]);
      obj.controller_data = controller_data;

					matdata.t = [];
					matdata.ee_curr = [];
					matdata.ee_goal = [];
					matdata.ee_err = [];
					matdata.ee_err_int = [];
					matdata.u_qp = [];
					matdata.u_cartesian = [];
					save('cartesian_err.mat','-struct','matdata','t','ee_curr','ee_goal','ee_err','ee_err_int','u_qp','u_cartesian');
    end

    function y = mimoOutput(obj,t,~,varargin)
				u_qp = varargin{1};
				x = varargin{2};
				ctrl_data = getData(obj.controller_data);
				
				typecheck(ctrl_data.ee_goal,'double');
				sizecheck(ctrl_data.ee_goal,[3,obj.num_rhand_pts+obj.num_lhand_pts]);
				ee_goal = ctrl_data.ee_goal;
				ee_controller_state = ctrl_data.ee_controller_state;
				ee_err_int = ctrl_data.ee_err_int;
				ee_prev_t = ctrl_data.ee_prev_t;
				r = obj.robot;
				q = x(1:obj.nq);
				qd = x(obj.nq+1:end);
				
				data = getNextMessage(obj.mon,0);
				if(~isempty(data))
					display('ManipCartesianController: receive robot plan');
					msg = drc.robot_plan_t(data);
					[xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true);
					q_end = xtraj(1:obj.nq,end);
					kinsol_goal = doKinematics(r,q_end);
					rh_goal_pos = forwardKin(r,kinsol_goal,obj.rhand_body,obj.rhand_pts,0);
					lh_goal_pos = forwardKin(r,kinsol_goal,obj.lhand_body,obj.lhand_pts,0);
				  ee_goal = [rh_goal_pos lh_goal_pos];	
					ee_controller_state = 1;
				end

				if(ee_controller_state == 1)
					kinsol_curr = doKinematics(r,q);
					[rh_curr,J_rh] = forwardKin(r,kinsol_curr,obj.rhand_body,obj.rhand_pts,0);
					[lh_curr,J_lh] = forwardKin(r,kinsol_curr,obj.lhand_body,obj.lhand_pts,0);
					ee_curr = [rh_curr lh_curr];
					ee_err = ee_goal-ee_curr;
					ee_err_norm = sum((sum(ee_err.*ee_err,1)));
					J_ee = 2*ee_err(:)'*[J_rh;J_lh];
					if(ee_err_norm<obj.cartesian_active_tol)
						ee_controller_state = 2;
					end
				end

				if(ee_controller_state == 0 || ee_controller_state == 1)
					u_cartesian = zeros(obj.nu,1);
				else
					display('ManipCartesianController: Proportional control on end effector');
					u_cartesian = obj.B_inv*J_ee(:,7:end)'*obj.Kp*ee_err_norm;
					matdata = load('cartesian_err.mat');
					matdata.t = [matdata.t t];
					matdata.ee_curr = [matdata.ee_curr ee_curr];
					matdata.ee_goal = [matdata.ee_goal ee_goal];
					matdata.ee_err = [matdata.ee_err ee_err];
					matdata.ee_err_int = [];
					matdata.u_qp = [matdata.u_qp u_qp];
					matdata.u_cartesian = [matdat.u_cartesian u_cartesian];
					save('cartesian_err.mat','-struct','matdata','t','ee_curr','ee_goal','ee_err','ee_err_int','u_qp','u_cartesian');
				end


% 			  rpalm_goal_data = getNextMessage(obj.rpalm_monitor,0);
% 				if(~isempty(rpalm_goal_data))
% 					display('ManipCartesianController: receive right hand goal');
% 					rpalm_goal_message = drc.ee_goal_t(rpalm_goal_data);
% 					rpalm_goal_pos = [rpalm_goal_message.ee_goal_pos.translation.x;rpalm_goal_message.ee_goal_pos.translation.y;rpalm_goal_message.ee_goal_pos.translation.z];
% 					rpalm_goal_quat = [rpalm_goal_message.ee_goal_pos.rotation.w;rpalm_goal_message.ee_goal_pos.rotation.x;rpalm_goal_message.ee_goal_pos.rotation.y;rpalm_goal_message.ee_goal_pos.rotation.z];
% 					T_world_palm_r = [quat2rotmat(rpalm_goal_quat) rpalm_goal_pos;0 0 0 1];
% 					T_world_hand_r = T_world_palm_r*obj.T_palm_hand_r;
% 					rh_goal_pos = T_world_hand_r*[obj.rhand_pts;ones(1,obj.num_rhand_pts)];
% 					rh_goal_pos = rh_goal_pos(1:3,:);
% 					rh_prev_t = t;
% 					rh_err_int = zeros(3,obj.num_rhand_pts);
% 					rh_controller_state = 1;
% 				end
% 				lpalm_goal_data = getNextMessage(obj.lpalm_monitor,0);
% 				if(~isempty(lpalm_goal_data))
% 					display('ManipCartesianController: receive left hand goal');
% 					lpalm_goal_message = drc.ee_goal_t(lpalm_goal_data);
% 					lpalm_goal_pos = [lpalm_goal_message.ee_goal_pos.translation.x;lpalm_goal_message.ee_goal_pos.translation.y;lpalm_goal_message.ee_goal_pos.translation.z];
% 					lpalm_goal_quat = [lpalm_goal_message.ee_goal_pos.rotation.w;lpalm_goal_message.ee_goal_pos.rotation.x;lpalm_goal_message.ee_goal_pos.rotation.y;lpalm_goal_message.ee_goal_pos.rotation.z];
% 					T_world_palm_l = [quat2rotmat(lpalm_goal_quat) lpalm_goal_pos;0 0 0 1];
% 					T_world_hand_l = T_world_palm_l*obj.T_palm_hand_l;
% 					lh_goal_pos = T_world_hand_l*[obj.lhand_pts;ones(1,obj.num_lhand_pts)];
% 					lh_goal_pos = lh_goal_pos(1:3,:);
% 					lh_prev_t = t;
% 					lh_err_int = zeros(3,obj.num_lhand_pts);
% 					lh_controller_state = 1;
% 				end
% 				
% 
% 				if(lh_controller_state == 0 && rh_controller_state == 0)
% 					u_cartesian = zeros(obj.nu,1);
% 				else
% 					kinsol = doKinematics(r,q);
% 					if(rh_controller_state ~=0)
% 						[rh_curr,J_rh] = forwardKin(r,kinsol,obj.rhand_body,obj.rhand_pts,0);
% 						rh_goal = rh_goal_pos;
% 						rh_err = rh_goal-rh_curr;
% 						if(rh_controller_state == 1)
% 							rh_err_norm = sum(sqrt(sum(rh_err.*rh_err,1)));
% 							if(rh_err_norm<obj.cartesian_active_tol)
% 								rh_controller_state = 2;
% 							end
% 						end
% 					end
% 					if(rh_controller_state == 2)
% 						display(sprintf('ManipCartesianController: right hand cartesian feedback is on at time %6.2fs',t));
% 						if(all(abs(rh_err_int)<obj.integral_bnd))
% 							rh_err_int = rh_err_int+rh_err*(t-rh_prev_t);
% 						end
% 						rh_total_err = reshape(obj.Kp*rh_err+obj.Ki*rh_err_int+obj.Kd*reshape(J_rh*qd,3,obj.num_rhand_pts),[],1);
% 					end
% 
% 					if(lh_controller_state ~=0)
% 						[lh_curr,J_lh] = forwardKin(r,kinsol,obj.lhand_body,obj.lhand_pts,0);
% 						lh_goal = lh_goal_pos;
% 						lh_err = lh_goal-lh_curr;
% 						if(lh_controller_state == 1)
% 							lh_err_norm = sum(sqrt(sum(lh_err.*lh_err,1)));
% 							if(lh_err_norm<obj.cartesian_active_tol)
% 								lh_controller_state = 2;
% 							end
% 						end
% 					end
% 					if(lh_controller_state == 2)
% 						display(sprintf('ManipCartesianController: left hand cartesian feedback is on at time %6.2fs',t));
% 						if(all(abs(lh_err_int)<obj.integral_bnd))
% 							lh_err_int = lh_err_int+lh_err*(t-lh_prev_t);
% 						end
% 						lh_total_err = reshape(obj.Kp*lh_err+obj.Ki*lh_err_int,[],1);
% 					end
% 
% 					if(rh_controller_state == 2 || lh_controller_state == 2)
% 						if(rh_controller_state == 2 && lh_controller_state == 2)
% 							ee_err = [rh_total_err;lh_total_err];
% 							J_ee = [J_rh;J_lh];
% 						elseif(rh_controller_state == 2)
% 							ee_err = rh_total_err;
% 							J_ee = J_rh;
% 						elseif(lh_controller-state == 2)
% 							ee_err = lh_total_err;
% 							J_ee = J_lh;
% 						end
% 						u_cartesian = obj.B_inv*J_ee(:,7:end)'*ee_err;
% 
% 						matdata = load('cartesian_err.mat');
% 						matdata.t = [matdata.t t];
% 						matdata.rh_curr = [matdata.rh_err rh_curr(:)];
% 						matdata.rh_goal = [matdata.rh_goal rh_goal_pos(:)];
% 						matdata.rh_err = [matdata.rh_err rh_err(:)];
% 						matdata.rh_err_int = [matdata.rh_err_int rh_err_int(:)];
% 						matdata.u_qp = [matdata.u_qp u_qp];
% 						matdata.u_cartesian = [matdata.u_cartesian u_cartesian];
% 						save('cartesian_err.mat','-struct','matdata','t','rh_curr','rh_goal','rh_err','rh_err_int','u_qp','u_cartesian');
% 					else
% 						u_cartesian = zeros(obj.nu,1);
% 					end
% 				end

				y = u_qp+u_cartesian;

    end
  end
end
