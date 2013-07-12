classdef PositionRefPublisher < DrakeSystem
  % sends joint position references and gains as joint commands to atlas
  
  properties
    act_idx;
    robot;
  end
  
  methods
    function obj = PositionRefPublisher(r,options)
      typecheck(r,'Atlas');
  
      if nargin<2
        options = struct();
      else
        typecheck(options,'struct');
      end
 
      if isfield(options,'gains_id')
        typecheck(options.gains_id,'char');
        posref = AtlasPositionRef(r,options.gains_id);
      else
        posref = AtlasPositionRef(r);
      end
  
      coords = AtlasCoordinates(r);
      obj = obj@DrakeSystem(0,0,coords.dim,posref.dim,true,true);
      obj = setInputFrame(obj,coords);
      obj = setOutputFrame(obj,posref);

      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.005;
      end
      
      obj.robot = r;
      obj.act_idx = getActuatedJoints(r);
      
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    end
   
    function y=output(obj,t,~,q_des)
      y = q_des(obj.act_idx);
    end
  end
  
end
