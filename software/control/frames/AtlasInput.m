classdef AtlasInput < LCMCoordinateFrameWCoder & Singleton
  % atlas input coordinate frame
  methods
    function obj=AtlasInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      coder = RobotInputCoder('atlas', input_names);
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasInput',r.getNumInputs(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(input_names);
      obj.setDefaultChannel('ACTUATOR_CMDS');
    end
  end
end
