classdef PosVelFeedForwardBlock < DrakeSystem
  
  methods
    function obj = PosVelFeedForwardBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      if (nargin<3) options=struct(); end
      
      nq = getNumDOF(r);
      nx = getNumStates(r);
      nu = getNumInputs(r);
      
      obj = obj@DrakeSystem(0,0,nx,3*nu,true,false);
      
      % check for the required fields in controller data
      fieldcheck(controller_data.data,'qtraj');
      fieldcheck(controller_data.data,'qdtraj');
      fieldcheck(controller_data.data,'qddtraj');
      fieldcheck(controller_data.data,'support_times');
      fieldcheck(controller_data.data,'supports');
      fieldcheck(controller_data.data,'ignore_terrain');
     
      % set controller data for QP controller
      setField(controller_data,'A',zeros(4));
      setField(controller_data,'B',zeros(4,2));
      setField(controller_data,'C',zeros(2,4));
      setField(controller_data,'Qy',zeros(2));
      setField(controller_data,'R',zeros(2));
      setField(controller_data,'is_time_varying',true);
      setField(controller_data,'S',zeros(4));
      setField(controller_data,'s1',ConstantTrajectory(zeros(4,1)));
      setField(controller_data,'s2',0);
      setField(controller_data,'x0',zeros(4,1));
      setField(controller_data,'u0',zeros(2,1));
      setField(controller_data,'trans_drift',zeros(3,1));
      setField(controller_data,'mu',1);
      setField(controller_data,'y0',ConstantTrajectory(zeros(2,1)));
        
      obj.ctrl_data = controller_data;
      
      % instantiate QP controller
      options.slack_limit = 30.0;
      options.w = 0.01;
      options.R = 1e-12*eye(nu);
      options.lcm_foot_contacts = false;
      options.full_body_opt = true;
      options.debug = false;
      if (isfield(options,'ignore_states'))
        % specifies what dimensions of the state we should ignore
        assert(isnumeric(options.ignore_states));
        obj.ignore_states = options.ignore_states;
        rmfield(options,'ignore_states');  % don't pass this to the QP controller
      else
        obj.ignore_states = [];
      end
      if ~isfield(options,'use_mex') options.use_mex = true; end

      obj.qp_controller = QPController(r,controller_data,options);
      
      obj = setInputFrame(obj,AtlasState(r));
      obj = setOutputFrame(obj,AtlasPositionRef(r,'crawling',4));
    end
    
    function y = output(obj,t,~,x)
      q = eval(obj.ctrl_data.data.qtraj,t);
      qdot = eval(obj.ctrl_data.data.qdtraj,t);
      qddot = eval(obj.ctrl_data.data.qddtraj,t);
      
      xt=[q;qdot];
      x(obj.ignore_states)=xt(obj.ignore_states);
      x(3)=-100;
      u = obj.qp_controller.mimoOutput(0,[],qddot,zeros(12,1),x);
      y = [q;qdot;u];
    end
    
  end
  
  properties
    ctrl_data;
    qp_controller;
    ignore_states;
  end
end