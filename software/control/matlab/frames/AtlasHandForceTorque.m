classdef AtlasHandForceTorque < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=AtlasHandForceTorque()
      coder = ContactStateCoder('atlas', 'l_hand');
      
      coords = {'fx','fy','fz','tx','ty','tz'};
      obj = obj@LCMCoordinateFrameWCoder('AtlasHandForceTorque',6,'f',JLCMCoder(coder));
      obj.setCoordinateNames(coords);
      obj.setDefaultChannel('EST_ROBOT_STATE');
    end
  end
end
