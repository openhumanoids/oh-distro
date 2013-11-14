classdef BDIManipCommandBlock < DrakeSystem
  % kindof a bunk little block. passes the robot coordinates and converts
  % planned motions in the pelvis to commands to the BDI manip mode
  % controller. these are published directly in the output function.  
  
  properties
    robot;
    params_pub;
  end
  
  methods
    function obj = BDIManipCommandBlock(r,options)
      typecheck(r,'Atlas');
  
      if nargin<2
        options = struct();
      else
        typecheck(options,'struct');
      end
      
      coords = AtlasCoordinates(r);
      obj = obj@DrakeSystem(0,0,coords.dim,coords.dim,true,true);
      obj = setInputFrame(obj,coords);
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

      obj.params_pub = AtlasManipParamsPublisher('ATLAS_MANIPULATE_PARAMS');
    end
   
    function y=output(obj,t,~,q_des)
      
      params.pelvis_height = max(obj.robot.pelvis_min_height, ...
        min(obj.robot.pelvis_max_height,getPelvisHeightAboveFeet(obj.robot,q_des)));
      params.pelvis_yaw = 0;
      params.pelvis_pitch = 0;
      params.pelvis_roll = 0;
      params.com_v0 = 0;
      params.com_v1 = 0;
      obj.param_pub.publish(params);
      
      y = q_des;
    end
  end
  
end
