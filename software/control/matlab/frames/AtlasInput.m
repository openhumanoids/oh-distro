classdef AtlasInput < LCMCoordinateFrameWCoder & Singleton
  % atlas input coordinate frame
  methods
    function obj=AtlasInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      coder = AtlasCommandCoder(input_names);
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasInput',r.getNumInputs(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(input_names);
      obj.setDefaultChannel('ATLAS_COMMAND');
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names);
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
    mex_ptr=0
  end
end
