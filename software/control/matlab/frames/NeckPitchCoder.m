classdef NeckPitchCoder < LCMCoder
  methods
    function d=dim(obj)
      d=1;
    end
    
    function str=timestampName(obj)
      str='utime';
    end
    
    function [x,t] = decode(obj,data)
      msg = drc.neck_pitch_t(data);
      x=msg.pitch;
      t=msg.utime / 1000000;
    end

    function msg = encode(obj,t,x)
      msg = drc.neck_pitch_t();
      msg.utime = 1000000*t;
      msg.pitch = x;
    end
  end
end
  