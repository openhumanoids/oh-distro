classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      coder = JointAnglesCoder('atlas',input_names);
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',r.getNumInputs(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(input_names);
      obj.setDefaultChannel('JOINT_POSITION_CMDS');
    end
  end
end
