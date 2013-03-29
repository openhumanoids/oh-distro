classdef PinnedAtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=PinnedAtlasPositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      [Kp,Kd,Ki] = getPIDGains(r,'pinnedtuned');
%       [Kp,Kd] = getPDGains(r,'gazebo');
      
      coder = JointCommandCoderWIntegralGains('atlas',input_names,diag(Kp),diag(Kd),diag(Ki));
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',r.getNumInputs(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(input_names);
      obj.setDefaultChannel('JOINT_COMMANDS');
    end
  end
end
