classdef BDIManipCommandBlock < MIMODrakeSystem
  % kindof a bunk little block. passes the robot coordinates and converts
  % planned motions in the pelvis to commands to the BDI manip mode
  % controller. these are published directly in the output function.  
  
  properties
    robot;
    params_pub;
    controller_data;
  end
  
  methods
    function obj = BDIManipCommandBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
  
      ctrl_data = getData(controller_data);
      if ~isfield(ctrl_data,'qtraj')
        error('QTrajEvalBlock: controller_data must contain qtraj field');
      end
      
      if nargin<3
        options = struct();
      else
        typecheck(options,'struct');
      end

      coords = AtlasCoordinates(r);
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.003;
      end
      
      obj.robot = r;
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
      obj.controller_data = controller_data;
      obj.params_pub = AtlasManipParamsPublisher('ATLAS_MANIPULATE_PARAMS');
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      q_des=varargin{1};
      x=varargin{2};

      if t<=obj.controller_data.data.qtraj.tspan(end)
        % only support pelvis height for the time being
        foot_z = getFootHeight(obj.robot,x(1:getNumDOF(obj.robot)));
        params.pelvis_height = max(obj.robot.pelvis_min_height, ...
          min(obj.robot.pelvis_max_height,q_des(3)-foot_z));
        params.pelvis_yaw = 0;
        params.pelvis_pitch = 0;
        params.pelvis_roll = 0;
        params.com_v0 = 0;
        params.com_v1 = 0;
        obj.params_pub.publish(params);
      end
      
      y = q_des;
    end
  end
  
end
