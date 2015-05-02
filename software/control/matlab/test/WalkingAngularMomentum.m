classdef WalkingAngularMomentum < atlasParams.Walking
  methods 
    function obj = WalkingAngularMomentum(varargin)
      obj = obj@atlasParams.Walking(varargin{:});
      obj.Kp_ang = 1.0;
      obj.W_kdot = 1e-5*eye(3);
    end
  end
end
