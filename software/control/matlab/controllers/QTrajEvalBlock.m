classdef QTrajEvalBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads evals qtraj at the current t
  properties
    robot;
    controller_data;
    integral_gains;
    dt;
  end
  
  methods
    function obj = QTrajEvalBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      ctrl_data = getData(controller_data);
      if ~isfield(ctrl_data,'qtraj')
        error('QTrajEvalBlock: controller_data must contain qtraj field');
      end
      
      if nargin<3
        options = struct();
      end
      
      if isfield(options,'integral_gains')
        % used to reduce steady state error (to e.g., deal with joint
        % backlash)
        typecheck(options.integral_gains,'double');
        sizecheck(options.integral_gains,[getNumDOF(r) 1]);
      else
        options.integral_gains = zeros(getNumDOF(r),1);
      end
      
      atlas_state = getStateFrame(r);
      input_frame = atlas_state;
      output_frame = MultiCoordinateFrame({AtlasCoordinates(r),atlas_state});
      
      obj = obj@MIMODrakeSystem(0,getNumDOF(r),input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.003;
      end
      obj = setSampleTime(obj,[obj.dt;0]);
      
      obj.robot = r;
      obj.controller_data = controller_data;
      obj.integral_gains = options.integral_gains;
    end
    
    function xn=mimoUpdate(obj,t,x,u)
      qtraj = obj.controller_data.data.qtraj;
      if isa(qtraj,'double')
        qdes=qtraj;
      else
        % pp trajectory
        qdes = fasteval(qtraj,t);
      end
      q=u(1:end/2);
      xn = x + obj.integral_gains.*(qdes-q)*obj.dt; % integral term
    end
       
    function [qdes,u]=mimoOutput(obj,t,x,u)
      qtraj = obj.controller_data.data.qtraj;
      if isa(qtraj,'double')
        qdes=qtraj;
      else
        % pp trajectory
        qdes = fasteval(qtraj,t);
      end
      qdes = qdes + x;
    end
  end
  
end
