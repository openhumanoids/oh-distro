classdef AtlasWalkingWrapper < DrakeSystem
  properties
    nq;
    nu;
    robot;
    input_map;
    controller_data;
    foot_contact_block;
    pd_plus_qp_block;
    velocity_int_block;
    qtraj_eval_block;
    footstep_plan_shift_block;
    pelvis_control_block;
    lfoot_control_block;
    rfoot_control_block;
  end
  
  methods
    function obj = AtlasWalkingWrapper(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'AtlasQPControllerData');
      
      if (~isfield(options, 'run_in_simul_mode'))
        options.run_in_simul_mode = false;
      end
      
      input_frame = getStateFrame(r);
      output_frame = AtlasPosVelTorqueRef(r);
      
      force_controlled_joints = controller_data.force_controlled_joints;
      position_controlled_joints = controller_data.position_controlled_joints;

      gains = getAtlasGains();
      gains.k_q_p(force_controlled_joints) = 0;
      gains.k_q_i(force_controlled_joints) = 0;
      gains.k_qd_p(force_controlled_joints) = 0;
      gains.k_f_p(position_controlled_joints) = 0;
      gains.ff_f_d(position_controlled_joints) = 0;
      gains.ff_qd(position_controlled_joints) = 0;
      gains.ff_qd_d(position_controlled_joints) = 0;

      output_frame.updateGains(gains);
      
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.nq = getNumPositions(r);
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
      
      % construct QP controller and related control blocks
      if (options.run_in_simul_mode)
        options.Kp_foot = [100; 100; 100; 150; 150; 150];
        options.foot_damping_ratio = 0.5;
        options.Kp_pelvis = [0; 0; 150; 200; 200; 200];
        options.pelvis_damping_ratio = 0.6;
        options.Kp_q = 150.0*ones(r.getNumPositions(),1);
        options.q_damping_ratio = 0.6;
      end
      [qp,lfoot_block,rfoot_block,pelvis_block,pd,options] = constructQPWalkingController(r,controller_data,options);
      obj.lfoot_control_block = lfoot_block;
      obj.rfoot_control_block = rfoot_block;
      obj.pelvis_control_block = pelvis_block;

      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 1;
      ins(3).input = 3;
      ins(4).system = 2;
      ins(4).input = 1;
      ins(5).system = 2;
      ins(5).input = 3;
      ins(6).system = 2;
      ins(6).input = 4;
      ins(7).system = 2;
      ins(7).input = 5;
      ins(8).system = 2;
      ins(8).input = 6;
      outs(1).system = 2;
      outs(1).output = 1;
      outs(2).system = 2;
      outs(2).output = 2;
      obj.pd_plus_qp_block = mimoCascade(pd,qp,[],ins,outs);
      clear ins;
    
      if (~options.run_in_simul_mode)
        options.use_error_integrator = true; % while we're still using position control in upper body
        options.use_lcm = true;
        options.use_contact_logic_OR = true;      
      else
        options.use_lcm = false;
        options.contact_threshold = 0.002;
      end
      obj.qtraj_eval_block = QTrajEvalBlock(r,controller_data,options);
      obj.foot_contact_block = FootContactBlock(r,controller_data,options);
      options.zero_ankles_on_contact = false;
      obj.velocity_int_block = VelocityOutputIntegratorBlock(r,options);
      obj.footstep_plan_shift_block = FootstepPlanShiftBlock(r,controller_data);

      controller_data.qd_int_state = zeros(obj.velocity_int_block.getStateFrame.dim,1);
      
      obj.robot = r;
      obj.input_map = getActuatedJoints(r);
      obj.controller_data = controller_data;
    end
   
    function y=output(obj,t,~,x)
      % foot contact
      fc = output(obj.foot_contact_block,t,[],x);

      % footstep plan shift
      junk = mimoOutput(obj.footstep_plan_shift_block,t,[],x,fc);
      
      % qtraj eval
      q_des_and_x = output(obj.qtraj_eval_block,t,[],x);
      q_des = q_des_and_x(1:obj.nq);
      
      % IK/QP
      lfoot_ddot = output(obj.lfoot_control_block,t,[],x);
      rfoot_ddot = output(obj.rfoot_control_block,t,[],x);
      pelvis_ddot = output(obj.pelvis_control_block,t,[],x);
      u_and_qdd = output(obj.pd_plus_qp_block,t,[],[q_des; x; fc; x; fc; lfoot_ddot; rfoot_ddot; pelvis_ddot]);
      u=u_and_qdd(1:obj.nu);
      qdd=u_and_qdd(obj.nu+(1:obj.nq));

      % velocity integrator
      qd_int_state = obj.controller_data.qd_int_state;
      qd_int_state = mimoUpdate(obj.velocity_int_block,t,qd_int_state,x,qdd,fc);
      obj.controller_data.qd_int_state = qd_int_state;
      qd_err = mimoOutput(obj.velocity_int_block,t,qd_int_state,x,qdd,fc);
    
      force_ctrl_joints = obj.controller_data.force_controlled_joints;
      qddes = 0*qd_err;
      qddes(force_ctrl_joints) = qd_err(force_ctrl_joints);
      udes = 0*u;
      udes(force_ctrl_joints) = u(force_ctrl_joints);
 
      y = [q_des(obj.input_map); qddes; udes];
    end
  end
  
end
