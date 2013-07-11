classdef AtlasCOM < LCMCoordinateFrameWCoder & Singleton
  % simple frame for Atlas COM
  methods
    function obj=AtlasCOM(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@LCMCoordinateFrameWCoder('COMGoal',3,'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)
        coder = drc.control.COMGoalCoder('atlas');
        
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        obj.setDefaultChannel('COM_GOAL');
      end
    end
  end
end
