classdef VehicleState < LCMCoordinateFrameWCoder & Singleton
  % state frame for the vehicle floating base
  methods
    function obj=VehicleState()
      coder = AffordanceFullStateCoder('car',[],[]);
      obj = obj@LCMCoordinateFrameWCoder('VehicleState',12,'x',JLCMCoder(coder));
      obj.setDefaultChannel('AFFORDANCE_COLLECTION');
    end
  end
end
