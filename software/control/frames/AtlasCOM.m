classdef AtlasCOM < LCMCoordinateFrameWCoder & Singleton
  % simple frame for Atlas COM
  methods
    function obj=AtlasCOM(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      coder = COMGoalCoder('atlas');
      obj = obj@LCMCoordinateFrameWCoder('COMGoal',3,'x',JLCMCoder(coder));
      obj.setDefaultChannel('COM_GOAL');
    end
  end
end
