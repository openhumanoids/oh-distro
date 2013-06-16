classdef NeckPitchFrame < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=NeckPitchFrame()

      obj = obj@LCMCoordinateFrameWCoder('NeckPitchFrame',1,'x');
      obj = obj@Singleton();
      if isempty(obj.lcmcoder)      
        coder = NeckPitchCoder();
        obj = setLCMCoder(obj,coder);

        obj.setCoordinateNames({'neck_pitch_world'});
        obj.setDefaultChannel('DESIRED_NECK_PITCH');
      end
    end
  end
end
