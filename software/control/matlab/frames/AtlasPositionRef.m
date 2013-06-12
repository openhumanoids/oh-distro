classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r,gains_id,send_mode)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      if (nargin<3) send_mode=2; end
      
      rangecheck(send_mode,2,4);
      if (send_mode==2) dim=getNumInputs(r);
      elseif (send_mode==3) dim=2*getNumInputs(r);
      elseif (send_mode==4) dim=3*getNumInputs(r);
      end
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',dim,'x');
      obj = obj@Singleton(['AtlasPositionRef_sendmode=',num2str(send_mode)]);
      if isempty(obj.lcmcoder)
        input_names = r.getInputFrame().coordinates;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix
      
        if nargin<2
          [Kp,Kd] = getPDGains(r,'gazebo');
        else
          typecheck(gains_id,'char');
          [Kp,Kd] = getPDGains(r,gains_id);
        end
      
        coder = AtlasCommandCoder(input_names,diag(Kp),diag(Kd),send_mode);
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        coords = input_names;
        if (send_mode>2)
          coords = vertcat(coords,cellfun(@(a) [a,'_dot'],input_names,'UniformOutput',false));
        end
        if (send_mode>3)
          coords = vertcat(coords,cellfun(@(a) [a,'_ff'],input_names,'UniformOutput',false));
        end
        obj.setCoordinateNames(coords);
        obj.setDefaultChannel('ATLAS_COMMAND');
      end
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names,diag(Kp),0*diag(Kd),send_mode);
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
