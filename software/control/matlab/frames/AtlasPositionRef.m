classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      [Kp,Kd] = getPDGains(r,'gazebo');
      
      coder = JointCommandCoder('atlas',input_names,diag(Kp),diag(Kd));
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',r.getNumInputs(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(input_names);
      obj.setDefaultChannel('JOINT_COMMANDS');
    end
  end
end
