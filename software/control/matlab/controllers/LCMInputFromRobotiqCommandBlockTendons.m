classdef LCMInputFromRobotiqCommandBlockTendons < LCMInputFromRobotiqCommandBlockBase
  properties
  end
  methods
    function obj = LCMInputFromRobotiqCommandBlockTendons(r, handedness, options)
      obj = obj@LCMInputFromRobotiqCommandBlockBase(r, handedness, options);
    end
    
    function varargout=mimoOutput(obj,t,~,hand_state)
      obj = obj.grabLCMMessage();
      varargout = {min(-obj.target_positions)};
    end
  end
  
end
