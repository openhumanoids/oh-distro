classdef FootMotionControlBlock < DrakeSystem

  properties
    nq;
    Kp;
    Kd;
    dt;
    controller_data; % pointer to shared data handle containing qtraj
    robot;
    foot_ind;
    mex_ptr;
    use_mex;
  end
  
  methods
    function obj = FootMotionControlBlock(r,name,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'QPControllerData');
      
      input_frame = getStateFrame(r);
      output_frame = BodySpatialAcceleration(r,name);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumPositions(r);

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

      if isfield(options,'use_mex')
        sizecheck(options.use_mex,1);
        obj.use_mex = uint32(options.use_mex);
        rangecheck(obj.use_mex,0,2);
        if (obj.use_mex && exist('bodyMotionControlmex','file')~=3)
          error('can''t find bodyMotionControlmex.  did you build it?');
        end
      else
        obj.use_mex = 1;
      end

      if (obj.use_mex>0)
        obj.mex_ptr = SharedDataHandle(bodyMotionControlmex(0,obj.robot.getMexModelPtr.ptr,obj.Kp,obj.Kd,obj.foot_ind));
      end

    end
   
    function y=output(obj,t,~,x)
      ctrl_data = obj.controller_data;
      link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.foot_ind;
      foot_traj_ind = find(ctrl_data.link_constraints(link_con_ind).ts<=t,1,'last');
      tt = t-ctrl_data.link_constraints(link_con_ind).ts(foot_traj_ind);
      a0 = ctrl_data.link_constraints(link_con_ind).a0(:,foot_traj_ind);
      a1 = ctrl_data.link_constraints(link_con_ind).a1(:,foot_traj_ind);
      a2 = ctrl_data.link_constraints(link_con_ind).a2(:,foot_traj_ind);
      a3 = ctrl_data.link_constraints(link_con_ind).a3(:,foot_traj_ind);
      [foot_des,foot_v_des,foot_vdot_des] = evalCubicSplineSegment(tt,a0,a1,a2,a3);
      foot_des = foot_des - [ctrl_data.plan_shift;0;0;0];
      % foot_vdot_des = [0;0;0;0;0;0];
      if (obj.use_mex == 0)
        q = x(1:obj.nq);
        qd = x(obj.nq+1:end);
        kinsol = doKinematics(obj.robot,q,false,true,qd);

        % TODO: this must be updated to use quaternions/spatial velocity
        [p,J] = forwardKin(obj.robot,kinsol,obj.foot_ind,[0;0;0],1); 

        err = [foot_des(1:3)-p(1:3);angleDiff(p(4:end),foot_des(4:end))];

        foot_vdot = obj.Kp.*err + obj.Kd.*(foot_v_des-J*qd) + foot_vdot_des; 
  		
      else
        foot_vdot = bodyMotionControlmex(obj.mex_ptr.data,x,foot_des,foot_v_des,foot_vdot_des);  
      end
      y = [obj.foot_ind;foot_vdot];
    end
  end
  
end
