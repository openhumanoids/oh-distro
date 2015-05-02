classdef StandingAngularMomentum < atlasParams.Standing
  methods
    function obj = StandingAngularMomentum(varargin);
      obj = obj@atlasParams.Standing(varargin{:});
      obj.Kp_ang = 1.0;
      obj.W_kdot = 1e-5*eye(3);
    end
  end
end
