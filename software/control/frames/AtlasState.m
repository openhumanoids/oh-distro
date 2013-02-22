classdef AtlasState < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      
      coder = RobotStateCoder('atlas', joint_names);
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(coder));
      obj.setCoordinateNames(r.getStateFrame.coordinates);
      obj.setDefaultChannel('TRUE_ROBOT_STATE');
    end
  end
end
