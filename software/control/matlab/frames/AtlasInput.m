classdef AtlasInput < LCMCoordinateFrameWCoder & Singleton
  % atlas input coordinate frame
  methods
    function obj=AtlasInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@LCMCoordinateFrameWCoder('AtlasInput',r.getNumInputs(),'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)
        input_names = r.getInputFrame().coordinates;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix
        
        mode = 1;
        gains = getAtlasGains(r,mode);
        
        coder = drc.control.AtlasCommandCoder(input_names,mode,gains.k_q_p,gains.k_q_i,...
          gains.k_qd_p,gains.k_f_p,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d,gains.ff_const);
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        obj.setCoordinateNames(input_names);
        obj.setDefaultChannel('ATLAS_COMMAND');
      end
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names,mode,gains.k_q_p,gains.k_q_i,...
          gains.k_qd_p,gains.k_f_p,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d,gains.ff_const);
        obj = setLCMCoder(obj,JLCMCoder(coder));
      end
    end
    
    function publish(obj,t,x,channel)
      % short-cut java publish with a faster mex version
      AtlasCommandPublisher(obj.mex_ptr,channel,t,x);
    end
    
    function delete(obj)
      AtlasCommandPublisher(obj.mex_ptr);
    end
  end
  
  properties
    mex_ptr=0;
  end
end
