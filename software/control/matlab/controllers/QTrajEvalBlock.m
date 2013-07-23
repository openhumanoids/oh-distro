classdef QTrajEvalBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads evals qtraj at the current t
  properties
    robot;
    controller_data;
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
      
      atlas_state = getStateFrame(r);
      input_frame = atlas_state;
      output_frame = MultiCoordinateFrame({AtlasCoordinates(r),atlas_state});
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.004;
      end
      obj = setSampleTime(obj,[dt;0]);
      
      obj.robot = r;
      obj.controller_data = controller_data;
    end
       
    function [qdes,x]=mimoOutput(obj,t,~,x)
      cdata = obj.controller_data.data;
      qtraj = cdata.qtraj;
      if typecheck(qtraj,'double')
        qdes=qtraj;
      else
        % pp trajectory
        qdes = fasteval(qtraj,t);
      end
    end
  end
  
end
