classdef VehicleState < LCMCoordinateFrameWCoder & Singleton
  % state frame for the vehicle floating base
  methods
    function obj=VehicleState()
      obj = obj@LCMCoordinateFrameWCoder('VehicleState',12,'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)
        coder = drc.control.AffordanceFullStateCoder('car',[],[]);
        obj = setLCMCoder(obj,JLCMCoder(coder));
        obj.setDefaultChannel('AFFORDANCE_COLLECTION');
      end
    end
  end
end
