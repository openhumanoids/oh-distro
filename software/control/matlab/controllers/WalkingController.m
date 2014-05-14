classdef WalkingController < DRCController

  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end

  methods
    function obj = WalkingController(name,r,options)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct(...
        'A',[zeros(2),eye(2); zeros(2,4)],...
        'B',[zeros(2); eye(2)],...
        'C',[eye(2),zeros(2)],...
        'Qy',eye(2),...
        'R',zeros(2),...
        'is_time_varying',false,...
        'S',zeros(4),...
        's1',zeros(4,1),...
        's2',0,...
        's1dot',zeros(4,1),...
        's2dot',0,...
        'x0',zeros(4,1),...
        'u0',zeros(2,1),...
        'y0',zeros(2,1),...
        'comtraj',[],...
        'link_constraints',[],...
        'trans_drift',[0;0;0],... % we subtract this value from the output of each foot trajectory to compensate for drift
        'support_times',[],...
        'supports',[],...
        'mu',1.0,...
        't_offset',0,...
        'ignore_terrain',false,...
        'qtraj',zeros(getNumDOF(r),1),...
        'xtraj',ConstantTrajectory(zeros(getNumStates(r),1)),... % used in QP controller if we ignore some subset of states
        'V',0,... % cost to go used in controller status message
        'Vdot',0)); % time derivative of cost to go used in controller status message

      if(~isfield(options,'use_mex')) options.use_mex = false; end
      if(~isfield(options,'debug')) options.debug = false; end
      if ~isfield(options,'lcm_foot_contacts') options.lcm_foot_contacts = true; end
      if ~isfield(options,'bipedal') options.bipedal = true; end

      if isfield(options,'use_walking_pd')
        typecheck(options.use_walking_pd, 'logical');
      else
        options.use_walking_pd = true;
      end

      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.001;
			options.contact_threshold = 0.005;
      qp = QPControlBlock(r,ctrl_data,options);

      if options.use_walking_pd
        % cascade walking PD controller
        pd = WalkingPDBlock(r,ctrl_data);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        ins(3).system = 2;
        ins(3).input = 3;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(pd,qp,[],ins,outs);
      else
        if ~isfield(options,'soft_ankles') options.soft_ankles = false; end
        % cascade simple PD controller
        pd = SimplePDBlock(r,ctrl_data,options);
        ins(1).system = 1;
        ins(1).input = 1;
        ins(2).system = 1;
        ins(2).input = 2;
        ins(3).system = 2;
        ins(3).input = 3;
        outs(1).system = 2;
        outs(1).output = 1;
        sys = mimoCascade(pd,qp,[],ins,outs);
      end
      clear connection ins outs;

      % cascade neck pitch control block
      neck = NeckControlBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 1;
      ins(2).input = 2;
      ins(3).system = 2;
      ins(3).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      connection(1).from_output = 1;
      connection(1).to_input = 1;
      connection(2).from_output = 2;
      connection(2).to_input = 2;
      sys = mimoCascade(neck,sys,connection,ins,outs);
      clear connection ins outs;

      % cascade footstep replanner
      fs = FootstepPlanShiftBlock(r,ctrl_data);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 1;
      ins(3).system = 2;
      ins(3).input = 3;
      outs(1).system = 2;
      outs(1).output = 1;
      connection.from_output = 1;
      connection.to_input = 2;
      sys = mimoCascade(fs,sys,connection,ins,outs);

      obj = obj@DRCController(name,sys,AtlasState(r));

      obj.controller_data = ctrl_data;

      if options.bipedal
        obj = setTimedTransition(obj,inf,'standing',false); % default timeout
        obj = addLCMTransition(obj,'STOP_WALKING',drc.plan_control_t(),'standing');
        obj = addLCMTransition(obj,'BRACE_FOR_FALL',drc.utime_t(),'bracing');
      end
    end

    function msg = status_message(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.WALKING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = obj.controller_data.getField('V');
        msg.Vdot = obj.controller_data.getField('Vdot');
    end

    function obj = initialize(obj,data)

      % TODO: put some error handling in here
      msg_data = data.WALKING_PLAN;

      walk_ctrl_data = WalkingControllerData.from_walking_plan_t(msg_data);
      obj.controller_data.setField('S',walk_ctrl_data.S);
      obj.controller_data.setField('s1',walk_ctrl_data.s1);

      istv = ~(isnumeric(walk_ctrl_data.s1) || isa(walk_ctrl_data.s1,'ConstantTrajectory'));
      obj.controller_data.setField('is_time_varying',istv);

      obj.controller_data.setField('s1dot',walk_ctrl_data.s1dot);

      obj.controller_data.setField('s2',walk_ctrl_data.s2);

      obj.controller_data.setField('s2dot',walk_ctrl_data.s2dot);

      support_times = msg_data.support_times;
      obj.controller_data.setField('support_times',support_times);

      obj.controller_data.setField('supports',walk_ctrl_data.supports);

      obj.controller_data.setField('comtraj',walk_ctrl_data.comtraj);

      obj.controller_data.setField('x0',[eval(walk_ctrl_data.zmptraj,walk_ctrl_data.zmptraj.tspan(2));0;0]);
      if ~istv
        assert(isa(walk_ctrl_data.zmptraj,'ConstantTrajectory'));
        obj.controller_data.setField('y0',eval(walk_ctrl_data.zmptraj,walk_ctrl_data.zmptraj.tspan(2)));
      else
        obj.controller_data.setField('y0',walk_ctrl_data.zmptraj);
      end
      tspan_end = walk_ctrl_data.zmptraj.tspan(end);

      obj.controller_data.setField('link_constraints',walk_ctrl_data.link_constraints);

      obj.controller_data.setField('qtraj',walk_ctrl_data.qtraj);

      obj.controller_data.setField('mu',msg_data.mu);

      % Reset the translational offset
      obj.controller_data.setField('trans_drift', [0;0;0]);

      obj = setDuration(obj,tspan_end,false); % set the controller timeout

      QPControlBlock.check_ctrl_data(obj.controller_data);
      delete(tmp_fname);
    end
  end
end
