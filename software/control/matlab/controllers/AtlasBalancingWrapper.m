classdef AtlasBalancingWrapper < DrakeSystem
  properties
    nq;
    nu;
    robot;
    input_map;
    ctrl_data;
    foot_contact_block;
    pd_plus_qp_block;
    velocity_int_block;
    qtraj_eval_block;
  end
  
  methods
    function obj = AtlasBalancingWrapper(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      input_frame = getStateFrame(r);
      output_frame = AtlasPosVelTorqueRef(r);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.nq = getNumDOF(r);
      obj.nu = getNumInputs(r);

      if nargin<3
        options = struct();
      end
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
      
      % instantiate QP controller
      options.slack_limit = 100;
      options.w_qdd = 10.0*ones(obj.nq,1);
      options.W_hdot = diag([10;10;10;10;10;10]);
      options.w_grf = 0.0075;
      options.w_slack = 0.005;
      options.Kp = 0; % com-z pd gains
      options.Kd = 0; % com-z pd gains
      options.input_foot_contacts = true;
      options.debug = false;
      options.use_mex = true;
      options.contact_threshold = 0.01;
      options.output_qdd = true;
      qp = MomentumControlBlock(r,{},controller_data,options);
     
      % cascade IK/PD block
      options.Kp = 80.0*ones(obj.nq,1);
      options.Kd = 12.0*ones(obj.nq,1);
      ankles = findJointIndices(r,'ak');
      options.Kp(ankles) = 20;
      options.Kd(ankles) = 3;
      pd = SimplePDBlock(r,controller_data,options);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 1;
      ins(4).system = 2;
      ins(4).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      outs(2).system = 2;
      outs(2).output = 2;
      obj.pd_plus_qp_block = mimoCascade(pd,qp,[],ins,outs);
      
      obj.qtraj_eval_block = QTrajEvalBlock(r,controller_data,options);
      obj.foot_contact_block = FootContactBlock(r);
      obj.velocity_int_block = VelocityOutputIntegratorBlock(r,options);

      if ~isfield(controller_data.data,'qtraj')
        error('AtlasBalancingWrapper: controller_data must contain qtraj field');
      end
      
      if ~isfield(controller_data.data,'qd_int_state')
        setField(controller_data,'qd_int_state',zeros(obj.velocity_int_block.getStateFrame.dim,1));
      else
        sizecheck(controller_data.qd_int_state,obj.velocity_int_block.getStateFrame.dim);
      end
      
      obj.robot = r;
      obj.input_map = getActuatedJoints(r);
      obj.ctrl_data = controller_data;
    end
   
    function y=output(obj,t,~,x)
      % foot contact
      fc = output(obj.foot_contact_block,t,[],x);
      
      % qtraj eval
      qtraj = obj.ctrl_data.data.qtraj;
      if isa(qtraj,'double')
        q_des=qtraj;
      else
        % pp trajectory
        q_des = fasteval(qtraj,t);
      end
      
      % IK/QP
      u_and_qdd = output(obj.pd_plus_qp_block,t,[],[q_des; x; x; fc]);
      u=u_and_qdd(1:obj.nu);
      qdd=u_and_qdd(obj.nu+(1:obj.nq));

      % velocity integrator
      qd_int_state = obj.ctrl_data.data.qd_int_state;
      qd_int_state = mimoUpdate(obj.velocity_int_block,t,qd_int_state,x,qdd,fc);
      setField(obj.ctrl_data,'qd_int_state',qd_int_state);
      qd_err = mimoOutput(obj.velocity_int_block,t,qd_int_state,x,qdd,fc);
    
      force_ctrl_joints = obj.ctrl_data.data.force_controlled_joints;
      qddes = 0*qd_err;
      qddes(force_ctrl_joints) = qd_err(force_ctrl_joints);
      udes = 0*u;
      udes(force_ctrl_joints) = u(force_ctrl_joints);
 
      y = [q_des(obj.input_map); qddes; udes];
    end
  end
  
end
