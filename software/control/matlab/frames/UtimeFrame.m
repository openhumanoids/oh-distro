classdef UtimeFrame < LCMCoordinateFrame & Singleton
  % dummy output frame for silent controller
  methods
    function obj=UtimeFrame()
      obj = obj@LCMCoordinateFrame('UtimeFrame',drc.dummy_control_t(),'x');
      obj = obj@Singleton('UtimeFrame');
      obj.setDefaultChannel('DUMMY_UTIME');
    end
  end
end
