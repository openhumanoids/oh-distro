classdef FootMotionControlBlock < DrakeSystem

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    foot_ind;
  end
  
  methods
    function obj = FootMotionControlBlock(r,name,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'SharedDataHandle');
      
      input_frame = getStateFrame(r);
      output_frame = BodySpatialAcceleration(r,name);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);

      if nargin<4
        options = struct();
      end
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[6 1]);
        obj.Kp = options.Kp;
      else
        obj.Kp = [100; 100; 100; 150; 150; 150];
      end        
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[6 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = [10; 10; 10; 10; 10; 10];
      end        
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.robot = r;
      obj.foot_ind = findLinkInd(r,name);
    end
   
    function y=output(obj,t,~,x)
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      kinsol = doKinematics(obj.robot,q,false,true,qd);

      % TODO: this must be updated to use quaternions/spatial velocity
      ctrl_data = obj.controller_data.data;
      [p,J] = forwardKin(obj.robot,kinsol,obj.foot_ind,[0;0;0],1); 
      
      % TODO: generate smooth footstep trajectories so we can incorporate
      % desired velocity and acceleration

      link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.foot_ind;
      foot_des = fasteval(ctrl_data.link_constraints(link_con_ind).traj,t);
      err = [foot_des(1:3)-p(1:3);angleDiff(p(4:end),foot_des(4:end))];
      
      foot_acc = obj.Kp.*err - obj.Kd.*(J*qd);
		%	foot_acc(4:5) = NaN;
      y = [obj.foot_ind;foot_acc];
    end
  end
  
end
