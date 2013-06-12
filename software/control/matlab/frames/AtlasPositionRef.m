classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r,gains_id)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',r.getNumInputs(),'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)
        input_names = r.getInputFrame().coordinates;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix
      
        if nargin<2
          [Kp,Kd] = getPDGains(r,'gazebo');
        else
          typecheck(gains_id,'char');
          [Kp,Kd] = getPDGains(r,gains_id);
        end
      
        coder = AtlasCommandCoder(input_names,diag(Kp),diag(Kd));
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        obj.setCoordinateNames(input_names);
        obj.setDefaultChannel('ATLAS_COMMAND');
      end
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names,diag(Kp),0*diag(Kd));
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
