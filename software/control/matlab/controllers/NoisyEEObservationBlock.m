classdef NoisyEEObservationBlock < MIMODrakeSystem
  properties
    robot;
    params;
  end
  
  methods
    function obj = NoisyEEObservationBlock(r,params)
      typecheck(r,'Atlas');
      typecheck(params,'struct');
      
      input_frame = getStateFrame(r);
      output_frame = params.ee.frame;
      
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
      obj.params = params;
    end
       
    function varargout=mimoOutput(obj,t,~,x)
      r = obj.robot;
      nq = getNumDOF(r); 
      q = x(1:nq); 
    
      kinsol = doKinematics(r,q,false);
      varargout = [];
      for i=1:length(obj.params)
        ee_pos = doKinematics(r,kinsol,obj.params(i).ee.body_index,...
          obj.params(i).ee.xyz_offset,obj.params(i).ee.include_rot);
        varargout(i) = ee_pos + obj.params(i).std*randn(size(ee_pos)); % just use white noise for now
      end
    end
  end
end
