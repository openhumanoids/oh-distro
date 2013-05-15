classdef NeckPitchFrame < LCMCoordinateFrameWCoder & Singleton
  
  methods
    function obj=NeckPitchFrame()

      coder = NeckPitchCoder();
      
      obj = obj@LCMCoordinateFrameWCoder('NeckPitchFrame',1,'x',coder);
      obj.setCoordinateNames({'neck_pitch_world'});
      obj.setDefaultChannel('DESIRED_NECK_PITCH');
    end
  end
end
