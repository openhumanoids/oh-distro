classdef FootContactState < CoordinateFrame & Singleton
  methods
    function obj=FootContactState()
      obj = obj@CoordinateFrame('drcFrames.FootContactState',2,'x',{'left','right'});
      obj = obj@Singleton();
    end
  end
end
