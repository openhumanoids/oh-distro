classdef PinnedAtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=PinnedAtlasPositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',r.getNumInputs(),'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)
        input_names = r.getInputFrame().coordinates;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
        [Kp,Kd,Ki] = getPIDGains(r,'pinned2');
%       [Kp,Kd] = getPDGains(r,'gazebo');
      
        coder = drc.control.JointCommandCoderWIntegralGains('atlas',input_names,diag(Kp),diag(Kd),diag(Ki));
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        obj.setCoordinateNames(input_names);
        obj.setDefaultChannel('JOINT_COMMANDS');
      end
    end
  end
end
