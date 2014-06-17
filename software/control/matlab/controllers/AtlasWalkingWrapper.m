classdef AtlasWalkingWrapper < DrakeSystem
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
    function obj = AtlasWalkingWrapper(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      input_frame = getStateFrame(r);
      output_frame = AtlasPosVelTorqueRef(r);
      
      force_controlled_joints = controller_data.data.force_controlled_joints;
      position_controlled_joints = controller_data.data.position_controlled_joints;

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
      
      use_simple_pd = false;
      constrain_torso = false;

      if use_simple_pd
        
        options.Kp = 30*ones(6,1);
        options.Kd = 8*ones(6,1);
        lfoot_motion = FootMotionControlBlock(r,'l_foot',controller_data,options);
        rfoot_motion = FootMotionControlBlock(r,'r_foot',controller_data,options);
        
        options.Kp = 30*[0; 0; 1; 1; 1; 0];
        options.Kd = 8*[0; 0; 1; 1; 1; 0];
        pelvis_motion = TorsoMotionControlBlock(r,'pelvis',controller_data,options);
        
        options.Kp = 40*[0; 0; 0; 1; 1; 1];
        options.Kd = 3*[0; 0; 0; 1; 1; 1];
        torso_motion = TorsoMotionControlBlock(r,'utorso',controller_data,options);
        
        options.w_qdd = 0.0001*ones(obj.nq,1);
        options.W_hdot = diag([1;1;1;10000;10000;10000]);
        options.w_grf = 0.01;
        options.Kp = 0; % com-z pd gains
        options.Kd = 0; % com-z pd gains
        options.body_accel_input_weights = [1 1 1 0];
      else
        options.w_qdd = 10*ones(obj.nq,1);
        options.W_hdot = diag([10;10;10;10;10;10]);
        options.w_grf = 0.0075;
        options.Kp = 0; % com-z pd gains
        options.Kd = 0; % com-z pd gains
      end

      % instantiate QP controller
      options.slack_limit = 50;
      options.w_slack = 0.005;
      options.input_foot_contacts = true;
      options.debug = false;
      options.use_mex = true;
      options.contact_threshold = 0.015;
      options.output_qdd = true;
      options.solver = 0;
      options.smooth_contacts = false;

      if use_simple_pd
        if constrain_torso
          motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame,pelvis_motion.getOutputFrame,torso_motion.getOutputFrame};
        else
          motion_frames = {lfoot_motion.getOutputFrame,rfoot_motion.getOutputFrame};
        end
        
        qp = MomentumControlBlock(r,motion_frames,ctrl_data,options);
        
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 2;
        ins(2).input = 1;
        ins(3).system = 2;
        ins(3).input = 2;
        ins(4).system = 2;
        ins(4).input = 3;
        ins(5).system = 2;
        ins(5).input = 5;
        if constrain_torso
          ins(6).system = 2;
          ins(6).input = 6;
          ins(7).system = 2;
          ins(7).input = 7;
        end
        outs(1).system = 2;
        outs(1).output = 1;
        outs(2).system = 2;
        outs(2).output = 2;
        qp = mimoCascade(lfoot_motion,qp,[],ins,outs);
        clear ins;
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 2;
        ins(2).input = 1;
        ins(3).system = 2;
        ins(3).input = 2;
        ins(4).system = 2;
        ins(4).input = 3;
        ins(5).system = 2;
        ins(5).input = 4;
        if constrain_torso
          ins(6).system = 2;
          ins(6).input = 6;
          ins(7).system = 2;
          ins(7).input = 7;
        end
        qp = mimoCascade(rfoot_motion,qp,[],ins,outs);
        if constrain_torso
          clear ins;
          ins(1).system = 1;
          ins(1).input = 1;
          ins(2).system = 2;
          ins(2).input = 1;
          ins(3).system = 2;
          ins(3).input = 2;
          ins(4).system = 2;
          ins(4).input = 3;
          ins(5).system = 2;
          ins(5).input = 4;
          ins(6).system = 2;
          ins(6).input = 5;
          ins(7).system = 2;
          ins(7).input = 7;
          qp = mimoCascade(pelvis_motion,qp,[],ins,outs);
          clear ins;
          ins(1).system = 1;
          ins(1).input = 1;
          ins(2).system = 2;
          ins(2).input = 1;
          ins(3).system = 2;
          ins(3).input = 2;
          ins(4).system = 2;
          ins(4).input = 3;
          ins(5).system = 2;
          ins(5).input = 4;
          ins(6).system = 2;
          ins(6).input = 5;
          ins(7).system = 2;
          ins(7).input = 6;
          qp = mimoCascade(torso_motion,qp,[],ins,outs);
        end
      else
        qp = MomentumControlBlock(r,{},ctrl_data,options);
      end
      vo = VelocityOutputIntegratorBlock(r,options);
      options.use_lcm = true;
      fcb = FootContactBlock(r,ctrl_data,options);
      fshift = FootstepPlanShiftBlock(r,ctrl_data);
      
      % cascade IK/PD block
      if use_simple_pd
        options.Kp = 40.0*ones(nq,1);
        options.Kd = 18.0*ones(nq,1);
        options.use_ik = false;
        pd = IKPDBlock(r,ctrl_data,options);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        ins(3).system = 2;
        ins(3).input = 1;
        ins(4).system = 2;
        ins(4).input = 2;
        ins(5).system = 2;
        ins(5).input = 3;
        if constrain_torso
          ins(6).system = 2;
          ins(6).input = 4;
          ins(7).system = 2;
          ins(7).input = 5;
          ins(8).system = 2;
          ins(8).input = 7;
        else
          ins(6).system = 2;
          ins(6).input = 5;
        end
      else
        options.Kp = 65.0*ones(nq,1);
        options.Kd = 14.5*ones(nq,1);
        options.Kp(findJointIndices(r,'hpz')) = 70.0;
        options.Kd(findJointIndices(r,'hpz')) = 14.0;
        options.Kd(findJointIndices(r,'kny')) = 13.0;
        options.Kp(3) = 30.0;
        options.Kd(3) = 12.0;
        options.Kp(4:5) = 30.0;
        options.Kd(4:5) = 12.0;
        options.Kp(6) = 40.0;
        options.Kd(6) = 12.0;
        pd = IKPDBlock(r,ctrl_data,options);
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
      end
      outs(1).system = 2;
      outs(1).output = 1;
      outs(2).system = 2;
      outs(2).output = 2;
      qp_sys = mimoCascade(pd,qp,[],ins,outs);
      clear ins;

udes = zeros(nu,1);
qddes = zeros(nu,1);
qd_int_state = zeros(nq+4,1);

qd_prev = zeros(nq,1);
qd_filt = zeros(nq,1);
eta = 0.1;
beta = ones(nq,1);

while tt<T
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    tau = x(2*nq+(1:nq));

    % low pass filter floating base velocities
    float_v = (1-alpha_v)*float_v + alpha_v*x(nq+(1:6));
    x(nq+(1:6)) = float_v;

    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    
%     zero_cross = sign(qd_prev) ~= sign(qd);
%     qd_prev = qd;
%     beta(zero_cross) = 0;
%     qd_filt = 0.1*(1-beta).*qd_filt + beta.*qd;
%     beta = min(1.0,beta+0.33);
%     
%     x_filt = [q;qd_filt];
    x_filt = [q;qd];

    fc = output(fcb,tt,[],x_filt);
  
    junk = output(fshift,tt,[],x_filt);

    if use_simple_pd
      if constrain_torso
        u_and_qdd = output(qp_sys,tt,[],[q0; x_filt; x_filt; x_filt; x_filt; x_filt; x_filt; fc]);
      else
        u_and_qdd = output(qp_sys,tt,[],[q0; x_filt; x_filt; x_filt; x_filt; fc]);
      end
    else
      u_and_qdd = output(qp_sys,tt,[],[q0; x_filt; fc; x_filt; fc]);
    end
    u=u_and_qdd(1:nu);
    qdd=u_and_qdd(nu+(1:nq));

    qd_int_state = mimoUpdate(vo,tt,qd_int_state,x_filt,qdd,fc);
    qd_ref = mimoOutput(vo,tt,qd_int_state,x_filt,qdd,fc);

    % fade in desired torques to avoid spikes at the start
    udes(joint_act_ind) = u(joint_act_ind);
    tau = tau(act_idx_map);
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);

    qddes(joint_act_ind) = qd_ref(joint_act_ind);

    ref_frame.publish(t,[q0(act_idx_map);qddes;udes],'ATLAS_COMMAND');
    %ref_frame.publish(t,[q(act_idx_map);qd_filt(act_idx_map);zeros(28,1)],'EST_ROBOT_STATE_KF');
  end
end
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      

     
      % cascade IK/PD block
      options.Kp = 65.0*ones(obj.nq,1);
      options.Kd = 12.0*ones(obj.nq,1);
      ankles = findJointIndices(r,'ak');
      options.Kp(ankles) = 20;
      options.Kd(ankles) = 3;
      options.use_ik = false;
      pd = IKPDBlock(r,controller_data,options);
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
      options.use_lcm = true;
      obj.foot_contact_block = FootContactBlock(r,controller_data,options);
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
        % ppform
        q_des = ppval(qtraj,min(t,qtraj.breaks(end)));
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
