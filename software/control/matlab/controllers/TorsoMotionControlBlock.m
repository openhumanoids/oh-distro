classdef TorsoMotionControlBlock < DrakeSystem

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    body_ind;
    rfoot_ind;
    lfoot_ind;
  end
  
  methods
    function obj = TorsoMotionControlBlock(r,name,controller_data,options)
      typecheck(r,'Atlas');
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
        obj.Kp = [0; 0; 0; 200; 200; 200];
      end        

      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[6 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = [0; 0; 0; 50; 50; 50];
      end        
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.002;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      obj.robot = r;
      obj.body_ind = findLinkInd(r,name);
      obj.lfoot_ind = findLinkInd(r,'l_foot');
      obj.rfoot_ind = findLinkInd(r,'r_foot');
    end
   
    function y=output(obj,t,~,x)
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      kinsol = doKinematics(obj.robot,q,false,true,qd); 
    
      % TODO: this must be updated to use quaternions/spatial velocity
      [p,J] = forwardKin(obj.robot,kinsol,obj.body_ind,[0;0;0],1); 
            
      % terrible hack
      lfoot = forwardKin(obj.robot,kinsol,obj.lfoot_ind,[0;0;0],1);
      rfoot = forwardKin(obj.robot,kinsol,obj.rfoot_ind,[0;0;0],1);
      
      body_des = [nan;nan;nan;0;0;mean([lfoot(6) rfoot(6)])]; 
      err = [body_des(1:3)-p(1:3);angleDiff(p(4:end),body_des(4:end))];

      body_vdot = obj.Kp.*err - obj.Kd.*(J*qd);
      y = [obj.body_ind;body_vdot];
    end
  end
  
end
