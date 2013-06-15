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

      obj.ctrl_data = controller_data;
     
%       % set controller data for QP controller
      qp_ctrl_data = SharedDataHandle(struct(...
          'A',zeros(4),...
          'B',zeros(4,2),...
          'C',zeros(2,4),...
          'Qy',zeros(2),...
          'R',zeros(2),...
          'is_time_varying',false,...
          'S',zeros(4),...
          's1',zeros(4,1),...
          's2',0,...
          'x0',zeros(4,1),...
          'u0',zeros(2,1),...
          'trans_drift',zeros(3,1),...
          'support_times',0,...
          'supports',SupportState(r,[]),...
          'mu',1,...
          'ignore_terrain',true,...
          'y0',zeros(2,1)));        
      
      % instantiate QP controller
      qp_options.slack_limit = 30.0;
      qp_options.w = 0.01;
      qp_options.R = 1e-12*eye(nu);
      qp_options.lcm_foot_contacts = false;
      qp_options.full_body_opt = true;
      qp_options.debug = false;
      qp_options.use_mex = 2;
      obj.robot = r;
%      if (isfield(options,'ignore_states'))
%        % specifies what dimensions of the state we should ignore
%        assert(isnumeric(options.ignore_states));
%        obj.ignore_states = qp_options.ignore_states;
%        rmfield(qp_options,'ignore_states');  % don't pass this to the QP controller
%      else
        obj.ignore_states = [];
%      end
      if ~isfield(options,'use_mex') qp_options.use_mex = true; end

      obj.qp_controller = QPController(r,qp_ctrl_data,qp_options);
      obj.actuated = getActuatedJoints(r);
      
      obj = setInputFrame(obj,AtlasState(r));
      obj = setOutputFrame(obj,AtlasPositionRef(r,'crawling',4));
%      [~,obj.robot] = inverseDynamics(r,zeros(getNumDOF(r),1),zeros(getNumDOF(r),1),zeros(getNumDOF(r),1),SupportState(r,[]));
    end
    
    function y = output(obj,t,~,x)
      q = eval(obj.ctrl_data.data.qtraj,t);
      qdot = eval(obj.ctrl_data.data.qdtraj,t);
      qddot = eval(obj.ctrl_data.data.qddtraj,t);
      
      xt=[q;qdot];
%       x(obj.ignore_states)=xt(obj.ignore_states);
%       x(3)=-100;
%      u = obj.qp_controller.mimoOutput(t,[],qddot,zeros(12,1),xt);

      if 0 % DEBUG MODE, PLOT COM
        kinsol = doKinematics(obj.robot,x(1:getNumDOF(r)));
        com = getCOM(obj.robot,kinsol);
        plot_lcm_points(com', [1 0 0], 660, 'COM Position', 1, true);
      end

      supp_idx = find(obj.ctrl_data.data.support_times<=t,1,'last');
      active_supports = obj.ctrl_data.data.supports(supp_idx);
      ctrl_data = obj.qp_controller.controller_data;
      setField(ctrl_data,'supports',active_supports);
      u = obj.qp_controller.mimoOutput(0,[],qddot,zeros(12,1),xt);
      
%      u = inverseDynamics(obj.robot,q,qdot,qddot,active_supports);
      y = [q(obj.actuated);qdot(obj.actuated);u];
    end
    
  end
  
  properties
    ctrl_data;
    qp_controller;
    ignore_states;
    actuated;
   robot;
  end
end